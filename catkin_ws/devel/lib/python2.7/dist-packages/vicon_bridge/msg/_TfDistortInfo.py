# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vicon_bridge/TfDistortInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TfDistortInfo(genpy.Message):
  _md5sum = "a7025291415a53264a70b836a949be8d"
  _type = "vicon_bridge/TfDistortInfo"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 tf_pub_rate
string tf_ref_frame
string tf_frame_in
string tf_frame_out
int32 delay
float64 position_scale
string noise_type
float64 sigma_roll_pitch
float64 sigma_yaw
float64 sigma_xy
float64 sigma_z
float64 random_walk_k_xy
float64 random_walk_k_z
float64 random_walk_sigma_xy
float64 random_walk_sigma_z
float64 random_walk_tau_xy
float64 random_walk_tau_z"""
  __slots__ = ['tf_pub_rate','tf_ref_frame','tf_frame_in','tf_frame_out','delay','position_scale','noise_type','sigma_roll_pitch','sigma_yaw','sigma_xy','sigma_z','random_walk_k_xy','random_walk_k_z','random_walk_sigma_xy','random_walk_sigma_z','random_walk_tau_xy','random_walk_tau_z']
  _slot_types = ['float64','string','string','string','int32','float64','string','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tf_pub_rate,tf_ref_frame,tf_frame_in,tf_frame_out,delay,position_scale,noise_type,sigma_roll_pitch,sigma_yaw,sigma_xy,sigma_z,random_walk_k_xy,random_walk_k_z,random_walk_sigma_xy,random_walk_sigma_z,random_walk_tau_xy,random_walk_tau_z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TfDistortInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.tf_pub_rate is None:
        self.tf_pub_rate = 0.
      if self.tf_ref_frame is None:
        self.tf_ref_frame = ''
      if self.tf_frame_in is None:
        self.tf_frame_in = ''
      if self.tf_frame_out is None:
        self.tf_frame_out = ''
      if self.delay is None:
        self.delay = 0
      if self.position_scale is None:
        self.position_scale = 0.
      if self.noise_type is None:
        self.noise_type = ''
      if self.sigma_roll_pitch is None:
        self.sigma_roll_pitch = 0.
      if self.sigma_yaw is None:
        self.sigma_yaw = 0.
      if self.sigma_xy is None:
        self.sigma_xy = 0.
      if self.sigma_z is None:
        self.sigma_z = 0.
      if self.random_walk_k_xy is None:
        self.random_walk_k_xy = 0.
      if self.random_walk_k_z is None:
        self.random_walk_k_z = 0.
      if self.random_walk_sigma_xy is None:
        self.random_walk_sigma_xy = 0.
      if self.random_walk_sigma_z is None:
        self.random_walk_sigma_z = 0.
      if self.random_walk_tau_xy is None:
        self.random_walk_tau_xy = 0.
      if self.random_walk_tau_z is None:
        self.random_walk_tau_z = 0.
    else:
      self.tf_pub_rate = 0.
      self.tf_ref_frame = ''
      self.tf_frame_in = ''
      self.tf_frame_out = ''
      self.delay = 0
      self.position_scale = 0.
      self.noise_type = ''
      self.sigma_roll_pitch = 0.
      self.sigma_yaw = 0.
      self.sigma_xy = 0.
      self.sigma_z = 0.
      self.random_walk_k_xy = 0.
      self.random_walk_k_z = 0.
      self.random_walk_sigma_xy = 0.
      self.random_walk_sigma_z = 0.
      self.random_walk_tau_xy = 0.
      self.random_walk_tau_z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.tf_pub_rate
      buff.write(_get_struct_d().pack(_x))
      _x = self.tf_ref_frame
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.tf_frame_in
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.tf_frame_out
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_id().pack(_x.delay, _x.position_scale))
      _x = self.noise_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.sigma_roll_pitch, _x.sigma_yaw, _x.sigma_xy, _x.sigma_z, _x.random_walk_k_xy, _x.random_walk_k_z, _x.random_walk_sigma_xy, _x.random_walk_sigma_z, _x.random_walk_tau_xy, _x.random_walk_tau_z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 8
      (self.tf_pub_rate,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tf_ref_frame = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tf_ref_frame = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tf_frame_in = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tf_frame_in = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tf_frame_out = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tf_frame_out = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.delay, _x.position_scale,) = _get_struct_id().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.noise_type = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.noise_type = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.sigma_roll_pitch, _x.sigma_yaw, _x.sigma_xy, _x.sigma_z, _x.random_walk_k_xy, _x.random_walk_k_z, _x.random_walk_sigma_xy, _x.random_walk_sigma_z, _x.random_walk_tau_xy, _x.random_walk_tau_z,) = _get_struct_10d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.tf_pub_rate
      buff.write(_get_struct_d().pack(_x))
      _x = self.tf_ref_frame
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.tf_frame_in
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.tf_frame_out
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_id().pack(_x.delay, _x.position_scale))
      _x = self.noise_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.sigma_roll_pitch, _x.sigma_yaw, _x.sigma_xy, _x.sigma_z, _x.random_walk_k_xy, _x.random_walk_k_z, _x.random_walk_sigma_xy, _x.random_walk_sigma_z, _x.random_walk_tau_xy, _x.random_walk_tau_z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 8
      (self.tf_pub_rate,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tf_ref_frame = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tf_ref_frame = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tf_frame_in = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tf_frame_in = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tf_frame_out = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tf_frame_out = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.delay, _x.position_scale,) = _get_struct_id().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.noise_type = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.noise_type = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.sigma_roll_pitch, _x.sigma_yaw, _x.sigma_xy, _x.sigma_z, _x.random_walk_k_xy, _x.random_walk_k_z, _x.random_walk_sigma_xy, _x.random_walk_sigma_z, _x.random_walk_tau_xy, _x.random_walk_tau_z,) = _get_struct_10d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10d = None
def _get_struct_10d():
    global _struct_10d
    if _struct_10d is None:
        _struct_10d = struct.Struct("<10d")
    return _struct_10d
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
_struct_id = None
def _get_struct_id():
    global _struct_id
    if _struct_id is None:
        _struct_id = struct.Struct("<id")
    return _struct_id
