"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class joint_state_t(object):
    __slots__ = ["aec", "pc", "vc", "iqc", "tau_c", "cmd", "timestamp_ms", "robot_id"]

    __typenames__ = ["float", "float", "float", "float", "float", "int16_t", "int64_t", "int64_t"]

    __dimensions__ = [[10], [10], [10], [10], [10], [10], None, None]

    def __init__(self):
        self.aec = [ 0.0 for dim0 in range(10) ]
        self.pc = [ 0.0 for dim0 in range(10) ]
        self.vc = [ 0.0 for dim0 in range(10) ]
        self.iqc = [ 0.0 for dim0 in range(10) ]
        self.tau_c = [ 0.0 for dim0 in range(10) ]
        self.cmd = [ 0 for dim0 in range(10) ]
        self.timestamp_ms = 0
        self.robot_id = 0

    def encode(self):
        buf = BytesIO()
        buf.write(joint_state_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>10f', *self.aec[:10]))
        buf.write(struct.pack('>10f', *self.pc[:10]))
        buf.write(struct.pack('>10f', *self.vc[:10]))
        buf.write(struct.pack('>10f', *self.iqc[:10]))
        buf.write(struct.pack('>10f', *self.tau_c[:10]))
        buf.write(struct.pack('>10h', *self.cmd[:10]))
        buf.write(struct.pack(">qq", self.timestamp_ms, self.robot_id))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != joint_state_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return joint_state_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = joint_state_t()
        self.aec = struct.unpack('>10f', buf.read(40))
        self.pc = struct.unpack('>10f', buf.read(40))
        self.vc = struct.unpack('>10f', buf.read(40))
        self.iqc = struct.unpack('>10f', buf.read(40))
        self.tau_c = struct.unpack('>10f', buf.read(40))
        self.cmd = struct.unpack('>10h', buf.read(20))
        self.timestamp_ms, self.robot_id = struct.unpack(">qq", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if joint_state_t in parents: return 0
        tmphash = (0x1dcc244112bdf876) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if joint_state_t._packed_fingerprint is None:
            joint_state_t._packed_fingerprint = struct.pack(">Q", joint_state_t._get_hash_recursive([]))
        return joint_state_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", joint_state_t._get_packed_fingerprint())[0]
