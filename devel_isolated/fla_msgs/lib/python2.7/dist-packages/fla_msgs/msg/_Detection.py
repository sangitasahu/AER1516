# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from fla_msgs/Detection.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Detection(genpy.Message):
  _md5sum = "f4c53395944a41874a6ab9783c6e93b6"
  _type = "fla_msgs/Detection"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint32 class_id
string class_name
float32 confidence

# (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates
float32 x_min 
float32 y_min
float32 x_max
float32 y_max

# ground truth unique identifier and position of the detected object (Simulation only) 
uint32 object_id
float32 x_pos
float32 y_pos
float32 z_pos
"""
  __slots__ = ['class_id','class_name','confidence','x_min','y_min','x_max','y_max','object_id','x_pos','y_pos','z_pos']
  _slot_types = ['uint32','string','float32','float32','float32','float32','float32','uint32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       class_id,class_name,confidence,x_min,y_min,x_max,y_max,object_id,x_pos,y_pos,z_pos

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Detection, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.class_id is None:
        self.class_id = 0
      if self.class_name is None:
        self.class_name = ''
      if self.confidence is None:
        self.confidence = 0.
      if self.x_min is None:
        self.x_min = 0.
      if self.y_min is None:
        self.y_min = 0.
      if self.x_max is None:
        self.x_max = 0.
      if self.y_max is None:
        self.y_max = 0.
      if self.object_id is None:
        self.object_id = 0
      if self.x_pos is None:
        self.x_pos = 0.
      if self.y_pos is None:
        self.y_pos = 0.
      if self.z_pos is None:
        self.z_pos = 0.
    else:
      self.class_id = 0
      self.class_name = ''
      self.confidence = 0.
      self.x_min = 0.
      self.y_min = 0.
      self.x_max = 0.
      self.y_max = 0.
      self.object_id = 0
      self.x_pos = 0.
      self.y_pos = 0.
      self.z_pos = 0.

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
      _x = self.class_id
      buff.write(_get_struct_I().pack(_x))
      _x = self.class_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_5fI3f().pack(_x.confidence, _x.x_min, _x.y_min, _x.x_max, _x.y_max, _x.object_id, _x.x_pos, _x.y_pos, _x.z_pos))
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
      end += 4
      (self.class_id,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.class_name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.class_name = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.confidence, _x.x_min, _x.y_min, _x.x_max, _x.y_max, _x.object_id, _x.x_pos, _x.y_pos, _x.z_pos,) = _get_struct_5fI3f().unpack(str[start:end])
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
      _x = self.class_id
      buff.write(_get_struct_I().pack(_x))
      _x = self.class_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_5fI3f().pack(_x.confidence, _x.x_min, _x.y_min, _x.x_max, _x.y_max, _x.object_id, _x.x_pos, _x.y_pos, _x.z_pos))
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
      end += 4
      (self.class_id,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.class_name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.class_name = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.confidence, _x.x_min, _x.y_min, _x.x_max, _x.y_max, _x.object_id, _x.x_pos, _x.y_pos, _x.z_pos,) = _get_struct_5fI3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5fI3f = None
def _get_struct_5fI3f():
    global _struct_5fI3f
    if _struct_5fI3f is None:
        _struct_5fI3f = struct.Struct("<5fI3f")
    return _struct_5fI3f
