"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class abb_irb140joints(object):
    __slots__ = ["utime", "pos", "vel"]

    def __init__(self):
        self.utime = 0
        self.pos = [ 0.0 for dim0 in range(6) ]
        self.vel = [ 0.0 for dim0 in range(6) ]

    def encode(self):
        buf = BytesIO()
        buf.write(abb_irb140joints._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        buf.write(struct.pack('>6d', *self.pos[:6]))
        buf.write(struct.pack('>6d', *self.vel[:6]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != abb_irb140joints._get_packed_fingerprint():
            raise ValueError("Decode error")
        return abb_irb140joints._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = abb_irb140joints()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.pos = struct.unpack('>6d', buf.read(48))
        self.vel = struct.unpack('>6d', buf.read(48))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if abb_irb140joints in parents: return 0
        tmphash = (0xdb4f922fdc668f51) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if abb_irb140joints._packed_fingerprint is None:
            abb_irb140joints._packed_fingerprint = struct.pack(">Q", abb_irb140joints._get_hash_recursive([]))
        return abb_irb140joints._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

