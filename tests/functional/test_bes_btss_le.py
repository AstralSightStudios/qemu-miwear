#!/usr/bin/env python3
"""Exercise the real BES controller through qtest mailbox and peer H4.

No host/application implementation is substituted. The ARM CPU is stopped by
qtest; one host drives the real intersys mailbox, the other the real chardev.
Usage: test_bes_btss_le.py --qemu build/qemu-system-arm --kernel vela_ap.bin
"""
import argparse
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import tempfile
import time
import unittest

U16 = lambda x: struct.pack('<H', x)
U32 = lambda x: struct.pack('<I', x)
BAND, PEER = 0, 1
BAND_ADDR = bytes.fromhex('010315eeffc0')
PEER_ADDR = bytes.fromhex('010000eeffc0')


class Controller:
    def __init__(self, qemu, kernel):
        self.tmp = tempfile.TemporaryDirectory(prefix='bes-btss-le-test-')
        root = Path(self.tmp.name)
        self.log = (root / 'qemu.log').open('w+')
        self.process = subprocess.Popen([
            str(qemu), '-M', f'miwear,profile=q66,nand-image={root}/nand.img',
            '-kernel', str(kernel), '-accel', 'qtest', '-display', 'none',
            '-serial', 'none', '-monitor', 'none', '-qtest-log', '/dev/null',
            '-qtest', f'unix:{root}/qtest,server=on,wait=off',
            '-chardev', f'socket,id=bt,path={root}/peer,server=on,wait=off',
            '-global', 'driver=bes.btss,property=chardev,value=bt',
            '-global', 'driver=bes.btss,property=latency-us,value=0',
            '-global', 'driver=bes.btss,property=reset-latency-us,value=0',
        ], stdout=self.log, stderr=self.log)
        self.qtest = self.connect(root / 'qtest')
        self.qfile = self.qtest.makefile('rwb', buffering=0)
        self.peer = self.connect(root / 'peer')
        self.peer.setblocking(False)
        self.buffer = bytearray()
        self.events = [[], []]
        self.write(0xc0004004, U32(0x20001000))
        self.write(0x20001000, U32(0x20001010))

    def connect(self, path):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.log.seek(0)
                raise RuntimeError(self.log.read())
            s = socket.socket(socket.AF_UNIX)
            try:
                s.connect(str(path))
                return s
            except (FileNotFoundError, ConnectionRefusedError):
                s.close()
                time.sleep(0.01)
        raise TimeoutError(str(path))

    def close(self):
        self.peer.close()
        self.qfile.close()
        self.qtest.close()
        self.process.terminate()
        self.process.wait(timeout=10)
        self.log.close()
        self.tmp.cleanup()

    def qt(self, text):
        self.qfile.write((text + '\n').encode())
        while True:
            reply = self.qfile.readline().decode().strip()
            if not reply:
                raise RuntimeError('QEMU qtest connection closed')
            if reply.startswith('IRQ '):
                continue
            if not reply.startswith('OK'):
                raise AssertionError((text, reply))
            return reply[2:].strip()

    def write(self, addr, data):
        self.qt(f'write {addr:#x} {len(data)} 0x{data.hex()}')

    def read(self, addr, length):
        return bytes.fromhex(self.qt(f'read {addr:#x} {length}')[2:])

    def word(self, addr):
        return struct.unpack('<I', self.read(addr, 4))[0]

    def band_send(self, data):
        self.write(0x20001010, struct.pack('<IB3xII', 0, 0, len(data), 0x20001100))
        self.write(0x20001100, data)
        self.qt('writel 0x400000a0 4')

    def send(self, side, data):
        if side == BAND:
            self.band_send(data)
        else:
            self.peer.sendall(data)

    def drain(self):
        while True:
            head = self.word(0xc0004000)
            desc = self.word(head) if head else 0
            if not desc:
                break
            while desc:
                header = self.read(desc, 16)
                nxt, _, length, ptr = struct.unpack('<IB3xII', header)
                self.events[BAND].append(self.read(ptr, length))
                desc = nxt
            self.qt('writel 0xd0330054 4')
        while select.select([self.peer], [], [], 0)[0]:
            chunk = self.peer.recv(65536)
            if not chunk:
                raise RuntimeError('H4 peer disconnected')
            self.buffer.extend(chunk)
        while self.buffer:
            if self.buffer[0] == 4:
                if len(self.buffer) < 3:
                    break
                length = self.buffer[2] + 3
            elif self.buffer[0] == 2:
                if len(self.buffer) < 5:
                    break
                length = int.from_bytes(self.buffer[3:5], 'little') + 5
            else:
                raise AssertionError(f'invalid H4 byte {self.buffer[0]:02x}')
            if len(self.buffer) < length:
                break
            self.events[PEER].append(bytes(self.buffer[:length]))
            del self.buffer[:length]

    def pump(self, ns=2_000_000):
        self.qt(f'clock_step {ns}')
        self.drain()

    def wait(self, side, predicate):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            self.pump()
            for i, event in enumerate(self.events[side]):
                if predicate(event):
                    return self.events[side].pop(i)
            select.select([self.peer], [], [], 0.002)
        raise AssertionError(f'expected event side={side}; queued={self.events}')

    def cmd(self, side, opcode, params=b'', status=0):
        self.send(side, b'\x01' + U16(opcode) + bytes([len(params)]) + params)
        event = self.wait(side, lambda e: e[0] == 4 and
            (e[1] == 0x0e and e[4:6] == U16(opcode) or
             e[1] == 0x0f and e[5:7] == U16(opcode)))
        actual = event[6] if event[1] == 0x0e else event[3]
        if actual != status:
            raise AssertionError((hex(opcode), status, event.hex()))
        return event

    def reset(self):
        for side in (BAND, PEER):
            self.cmd(side, 0x0c03)
            self.cmd(side, 0x0c01, b'\xff' * 8)
            self.cmd(side, 0x2001, b'\xff' * 8)
        self.pump()
        self.events = [[], []]

    def adv(self, side=PEER, own=0, kind=0, target_type=0, target=BAND_ADDR):
        self.cmd(side, 0x2006, struct.pack('<HHBBB6sBB', 160, 160, kind,
                                        own, target_type, target, 7, 0))
        self.cmd(side, 0x200a, b'\x01')

    def create(self, side=BAND, addr=PEER_ADDR, kind=0, own=0, extended=False,
               policy=0, status=0):
        block = struct.pack('<HHHHHHHH', 48, 48, 24, 40, 0, 500, 0, 0)
        if extended:
            params = bytes([policy, own, kind]) + addr + b'\x01' + block
            opcode = 0x2043
        else:
            params = block[:4] + bytes([policy, kind]) + addr + bytes([own]) + block[4:]
            opcode = 0x200d
        return self.cmd(side, opcode, params, status)

    def complete(self, side, sub=0x0a, status=0):
        event = self.wait(side, lambda e: e[:2] == b'\x04\x3e' and e[3] == sub)
        assert event[4] == status, event.hex()
        assert event[2] == (31 if sub == 0x0a else 19), event.hex()
        return event

    def link(self, central=BAND, extended=False):
        self.adv(central ^ 1)
        self.create(central, PEER_ADDR if central == BAND else BAND_ADDR,
                    extended=extended)
        a = self.complete(central)
        b = self.complete(central ^ 1)
        assert int.from_bytes(a[5:7], 'little') == 2
        assert a[7] == 0 and b[7] == 1
        return a, b


class LeControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = Controller(ARGS.qemu, ARGS.kernel)

    @classmethod
    def tearDownClass(cls):
        cls.c.close()

    def setUp(self):
        self.c.reset()

    def test_public_connection_both_roles_and_event_fields(self):
        for central in (BAND, PEER):
            with self.subTest(central=central):
                self.c.reset()
                a, b = self.c.link(central)
                self.assertEqual(a[8], 0)
                self.assertEqual(a[9:15], PEER_ADDR if central == BAND else BAND_ADDR)
                self.assertEqual(a[15:27], bytes(12))
                self.assertEqual(struct.unpack('<HHH', a[27:33]), (24, 0, 500))
                self.assertEqual(a[33], 0)
                self.assertEqual(b[7], 1)

    def test_pending_wrong_peer_then_cancel_ordering(self):
        self.c.adv()
        self.c.create(addr=b'\x55' * 6)
        self.c.pump(1_000_000_000)
        self.assertFalse(any(e[:2] == b'\x04\x3e' for e in self.c.events[BAND]))
        self.c.create(status=0x0c)
        self.c.cmd(BAND, 0x200e)
        self.c.complete(BAND, status=2)
        self.c.cmd(BAND, 0x200e, status=0x0c)
        self.c.pump(1_000_000_000)
        self.assertFalse(any(e[:2] == b'\x04\x3e' for e in self.c.events[BAND]))

    def test_late_advertiser_connects_pending_initiator(self):
        self.c.create()
        self.c.pump(1_000_000_000)
        self.c.adv()
        self.c.complete(BAND)
        self.c.complete(PEER)

    def test_nonconnectable_and_directed_mismatch_stay_pending(self):
        for kind in (3, 1):
            with self.subTest(kind=kind):
                self.c.reset()
                self.c.adv(kind=kind, target=b'\x55' * 6)
                self.c.create()
                self.c.pump(1_000_000_000)
                self.assertFalse(any(e[:2] == b'\x04\x3e' for e in self.c.events[BAND]))
                self.c.cmd(BAND, 0x200e)
                self.c.complete(BAND, status=2)

    def test_directed_advertiser_matching_peer_connects(self):
        self.c.adv(kind=4)
        self.c.create()
        self.c.complete(BAND)
        self.c.complete(PEER)

    def test_legacy_mask_and_enhanced_precedence(self):
        self.c.cmd(BAND, 0x2001, U32(1) + U32(0))
        self.c.adv()
        self.c.create()
        self.c.complete(BAND, sub=1)
        self.c.complete(PEER, sub=0x0a)
        self.assertFalse(any(e[:4] == b'\x04\x3e\x13\x01' for e in self.c.events[PEER]))

    def test_masks_do_not_change_link_establishment(self):
        self.c.cmd(BAND, 0x2001, bytes(8))
        self.c.adv()
        self.c.create()
        self.c.complete(PEER)
        self.c.pump()
        self.assertFalse(any(e[:2] == b'\x04\x3e' for e in self.c.events[BAND]))
        self.c.cmd(BAND, 0x0406, U16(2) + b'\x13')

    def test_random_address_type_matching(self):
        random = bytes.fromhex('1234567890c1')
        self.c.cmd(PEER, 0x2005, random)
        self.c.adv(own=1)
        self.c.create(addr=random, kind=0)
        self.c.pump(1_000_000_000)
        self.c.cmd(BAND, 0x200e)
        self.c.complete(BAND, status=2)
        self.c.create(addr=random, kind=1)
        event = self.c.complete(BAND)
        self.assertEqual(event[8], 1)
        self.assertEqual(event[9:15], random)
        self.c.complete(PEER)

    def test_random_own_address_requires_configuration(self):
        self.c.create(own=1, status=0x12)
        self.c.cmd(BAND, 0x2005, bytes.fromhex('1122334455c0'))
        self.c.adv()
        self.c.create(own=1)
        event = self.c.complete(PEER)
        self.assertEqual(event[8], 1)
        self.assertEqual(event[9:15], bytes.fromhex('1122334455c0'))

    def test_malformed_create_never_leaves_pending_state(self):
        valid = struct.pack('<HHBB6sBHHHHHH', 48, 48, 0, 0, PEER_ADDR,
                            0, 24, 40, 0, 500, 0, 0)
        for params in (b'', valid[:-1], valid + b'\0',
                       b'\x04\0\x05\0' + valid[4:],
                       valid[:13] + U16(5) + valid[15:],
                       valid[:19] + U16(10) + valid[21:]):
            with self.subTest(params=params.hex()):
                self.c.cmd(BAND, 0x200d, params, status=0x12)
                self.c.cmd(BAND, 0x200e, status=0x0c)

    def test_extended_create_and_bad_phy_layout(self):
        self.c.link(extended=True)
        self.c.cmd(BAND, 0x0406, U16(2) + b'\x13')
        prefix = b'\x00\x00\x00' + PEER_ADDR
        block = struct.pack('<HHHHHHHH', 48, 48, 24, 40, 0, 500, 0, 0)
        for mask, data, status in [(0,b'',0x12), (8,block,0x12),
                                   (3,block,0x12), (3,block*2,0x11)]:
            self.c.cmd(BAND, 0x2043, prefix + bytes([mask]) + data, status=status)

    def test_filter_accept_list_controls_initiation(self):
        self.c.adv()
        self.c.create(policy=1)
        self.c.pump(1_000_000_000)
        self.c.cmd(BAND, 0x2011, b'\0' + PEER_ADDR, status=0x0c)
        self.c.cmd(BAND, 0x200e)
        self.c.complete(BAND, status=2)
        self.c.cmd(BAND, 0x2011, b'\0' + PEER_ADDR)
        self.c.create(policy=1)
        self.c.complete(BAND)
        self.c.complete(PEER)

    def test_scan_reports_use_actual_enabled_advertiser(self):
        self.c.cmd(BAND, 0x200b, b'\x01\x10\0\x10\0\0\0')
        self.c.cmd(PEER, 0x2008, b'\x03\x02\x01\x02' + bytes(28))
        self.c.adv()
        self.c.cmd(BAND, 0x200c, b'\x01\x01')
        e = self.c.wait(BAND, lambda e:e[:4] == b'\x04\x3e\x0f\x02')
        self.assertEqual(e[7:13], PEER_ADDR)
        self.assertEqual(e[14:17], b'\x02\x01\x02')

    def test_le_acl_both_directions_preserves_pb_and_completion_handles(self):
        self.c.link()
        for side in (BAND, PEER):
            for pb in (0, 1, 2):
                packet = b'\x02' + U16(2 | pb << 12) + U16(7) + b'\x03\0\x05\0abc'
                self.c.send(side, packet)
                actual = self.c.wait(side ^ 1, lambda e:e[0] == 2)
                self.assertEqual(actual, packet)
                completed = self.c.wait(side, lambda e:e[:2] == b'\x04\x13')
                self.assertEqual(completed[3:], b'\x01\x02\0\x01\0')

    def test_peer_acl_respects_band_host_buffer_fragmentation(self):
        self.c.link()
        self.c.cmd(BAND, 0x0c33, U16(4) + b'\0' + U16(8) + U16(0))
        packet = b'\x02' + U16(0x2002) + U16(9) + b'123456789'
        self.c.send(PEER, packet)
        packets = [self.c.wait(BAND, lambda e:e[0] == 2) for _ in range(3)]
        self.assertEqual([int.from_bytes(e[1:3], 'little') for e in packets], [0x2002,0x1002,0x1002])
        self.assertEqual(b''.join(e[5:] for e in packets), b'123456789')
        self.c.wait(PEER, lambda e:e[:2] == b'\x04\x13')

    def test_disconnect_unknown_and_invalid_reason_do_not_drop_le(self):
        self.c.link()
        self.c.cmd(BAND, 0x0406, U16(3) + b'\x13', status=2)
        self.c.cmd(BAND, 0x0406, U16(2) + b'\xff', status=0x12)
        self.c.cmd(BAND, 0x0406, U16(2) + b'\x13')
        a = self.c.wait(BAND, lambda e:e[:2] == b'\x04\x05')
        b = self.c.wait(PEER, lambda e:e[:2] == b'\x04\x05')
        self.assertEqual(a[3:], b'\0\x02\0\x16')
        self.assertEqual(b[3:], b'\0\x02\0\x13')
        self.c.link()

    def test_remote_features_version_and_connection_update(self):
        self.c.link()
        self.c.cmd(BAND, 0x2016, U16(2))
        e = self.c.wait(BAND, lambda e:e[:4] == b'\x04\x3e\x0c\x04')
        self.assertEqual(e[4:], b'\0\x02\0' + bytes(8))
        self.c.cmd(PEER, 0x041d, U16(2))
        e = self.c.wait(PEER, lambda e:e[:2] == b'\x04\x0c')
        self.assertEqual(e[3:6], b'\0\x02\0')
        self.c.cmd(BAND, 0x2013, U16(2) + struct.pack('<HHHHHH', 32,40,0,500,0,0))
        for side in (BAND, PEER):
            e = self.c.wait(side, lambda e:e[:4] == b'\x04\x3e\x0a\x03')
            self.assertEqual(e[5:], U16(2) + struct.pack('<HHH',32,0,500))
        self.c.cmd(BAND, 0x2013, U16(3) + struct.pack('<HHHHHH',32,40,0,500,0,0),status=2)
        self.c.cmd(BAND, 0x2032, U16(2)+bytes(5), status=0x11)

    def test_classic_and_le_links_are_independent(self):
        self.c.cmd(PEER, 0x0c1a, b'\x02')
        self.c.cmd(BAND, 0x0405, PEER_ADDR + U16(0xcc18) + b'\x01\0' + U16(0) + b'\x01')
        self.c.wait(PEER, lambda e:e[:2] == b'\x04\x04')
        self.c.cmd(PEER, 0x0409, BAND_ADDR + b'\x01')
        for side in (BAND, PEER):
            e = self.c.wait(side, lambda e:e[:2] == b'\x04\x03')
            self.assertEqual(e[3:6], b'\0\x01\0')
        self.c.link()
        for handle in (1,2):
            for side in (BAND,PEER):
                packet = b'\x02'+U16(0x2000|handle)+U16(5)+bytes([handle])*5
                self.c.send(side,packet)
                self.assertEqual(self.c.wait(side^1,lambda e:e[0]==2),packet)
                e=self.c.wait(side,lambda e:e[:2]==b'\x04\x13')
                self.assertEqual(e[4:6], U16(handle))
        self.c.cmd(BAND,0x0406,U16(2)+b'\x13')
        for side in (BAND,PEER):
            self.c.wait(side,lambda e:e[:2]==b'\x04\x05')
        packet=b'\x02'+U16(0x2001)+U16(5)+b'alive'
        self.c.send(BAND,packet)
        self.assertEqual(self.c.wait(PEER,lambda e:e[0]==2),packet)
        self.c.cmd(BAND,0x0406,U16(1)+b'\x13')

    def test_reset_tears_down_le_and_cancels_only_own_initiation(self):
        self.c.link()
        self.c.cmd(BAND,0x0c03)
        e=self.c.wait(PEER,lambda e:e[:2]==b'\x04\x05')
        self.assertEqual(e[3:],b'\0\x02\0\x08')
        self.c.cmd(BAND,0x0c01,b'\xff'*8)
        self.c.cmd(BAND,0x2001,b'\xff'*8)
        self.c.create()
        self.c.cmd(BAND,0x0c03)
        self.c.adv()
        self.c.pump(1_000_000_000)
        self.assertFalse(any(e[:2]==b'\x04\x3e' for e in self.c.events[BAND]))


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--qemu',type=Path,required=True)
    parser.add_argument('--kernel',type=Path,required=True)
    ARGS,rest=parser.parse_known_args()
    ARGS.qemu=ARGS.qemu.resolve(strict=True)
    ARGS.kernel=ARGS.kernel.resolve(strict=True)
    unittest.main(argv=[__file__]+rest)
