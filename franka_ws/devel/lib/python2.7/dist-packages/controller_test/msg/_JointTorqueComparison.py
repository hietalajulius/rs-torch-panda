# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from controller_test/JointTorqueComparison.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class JointTorqueComparison(genpy.Message):
  _md5sum = "6c09db90263c92a2e4e4d736f67bc033"
  _type = "controller_test/JointTorqueComparison"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64[7] tau_error
float64[7] tau_commanded
float64[7] tau_measured
float64 root_mean_square_error
"""
  __slots__ = ['tau_error','tau_commanded','tau_measured','root_mean_square_error']
  _slot_types = ['float64[7]','float64[7]','float64[7]','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tau_error,tau_commanded,tau_measured,root_mean_square_error

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(JointTorqueComparison, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.tau_error is None:
        self.tau_error = [0.] * 7
      if self.tau_commanded is None:
        self.tau_commanded = [0.] * 7
      if self.tau_measured is None:
        self.tau_measured = [0.] * 7
      if self.root_mean_square_error is None:
        self.root_mean_square_error = 0.
    else:
      self.tau_error = [0.] * 7
      self.tau_commanded = [0.] * 7
      self.tau_measured = [0.] * 7
      self.root_mean_square_error = 0.

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
      buff.write(_get_struct_7d().pack(*self.tau_error))
      buff.write(_get_struct_7d().pack(*self.tau_commanded))
      buff.write(_get_struct_7d().pack(*self.tau_measured))
      _x = self.root_mean_square_error
      buff.write(_get_struct_d().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 56
      self.tau_error = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 56
      self.tau_commanded = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 56
      self.tau_measured = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 8
      (self.root_mean_square_error,) = _get_struct_d().unpack(str[start:end])
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
      buff.write(self.tau_error.tostring())
      buff.write(self.tau_commanded.tostring())
      buff.write(self.tau_measured.tostring())
      _x = self.root_mean_square_error
      buff.write(_get_struct_d().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 56
      self.tau_error = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=7)
      start = end
      end += 56
      self.tau_commanded = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=7)
      start = end
      end += 56
      self.tau_measured = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=7)
      start = end
      end += 8
      (self.root_mean_square_error,) = _get_struct_d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
