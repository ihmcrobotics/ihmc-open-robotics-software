// generated from rosidl_generator_dds_idl/resource/msg.idl.em

@###############################################
@#
@# ROS interface to DDS interface converter
@#
@# EmPy template for generating <msg>.idl files
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - subfolders (list of strings)
@#    The subfolders under the package name
@#    in which the type gets defined which are not part of the namespace
@#  - get_include_directives (function)
@#  - msg_type_to_idl (function)
@###############################################
@
@{
from rosidl_generator_cpp import escape_string
from rosidl_generator_dds_idl import MSG_TYPE_TO_IDL
from rosidl_generator_dds_idl import _msg_type_to_idl

def msg_type_to_idl_2(type_):
    """
    Convert a message type into the DDS declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>

    @param type: The message type
    @type type: rosidl_parser.Type
    """
    string_upper_bound = None
    if type_.is_primitive_type():
        idl_type = MSG_TYPE_TO_IDL[type_.type]
        if type_.type == 'string' and type_.string_upper_bound is not None:
            string_upper_bound = type_.string_upper_bound
    else:
        if type_.type.endswith('_Request') or type_.type.endswith('_Response'):
            idl_type = '%s::srv::dds::%s' % (type_.pkg_name, type_.type)
        else:
            idl_type = '%s::msg::dds::%s' % (type_.pkg_name, type_.type)
    return _msg_type_to_idl(type_, idl_type, string_upper_bound=string_upper_bound)

}@
@
#ifndef __@(spec.base_type.pkg_name)__@(subfolder)__@(spec.base_type.type)__idl__
#define __@(spec.base_type.pkg_name)__@(subfolder)__@(spec.base_type.type)__idl__

@#############################
@# Include dependency messages
@#############################
@[if spec.base_type.type.startswith('Sample_')]
@[  for line in get_include_directives(spec, ['srv'] + subfolders)]@
@(line)
@[  end for]@
@[else]@
@[  for line in get_include_directives(spec, ['msg'] + subfolders)]@
@(line)
@[  end for]@
@[end if]@

module @(spec.base_type.pkg_name)
{

module @(subfolder)
{

module dds
{

@##################
@# Define constants
@##################
@# Constants
@[for constant in spec.constants]@
  const @(MSG_TYPE_TO_IDL[constant.type]) @(spec.base_type.type)__@(constant.name) =
@[  if constant.type == 'bool']@
    @('TRUE' if constant.value else 'FALSE');
@[  elif constant.type == 'char']@
    '\@(constant.value)';
@[  elif constant.type == 'int8']@
    @(constant.value if constant.value >= 0 else (constant.value + 256));
@[  elif constant.type == 'string']@
    "@(escape_string(constant.value))";
@[  else]@
    @(constant.value);
@[  end if]@
@[end for]

@{
typedefs = set([])
for field in spec.fields:
  idl_typedef, idl_typedef_var, _ = msg_type_to_idl_2(field.type)
  if idl_typedef and idl_typedef_var and (idl_typedef, idl_typedef_var) not in typedefs:
    print('%s %s__%s__%s' % (idl_typedef, spec.base_type.pkg_name, spec.base_type.type, idl_typedef_var))
    typedefs.add((idl_typedef, idl_typedef_var))
}@

@################################
@# Message struct with all fields
@################################
@@TypeCode(type="@(spec.base_type.pkg_name)::@(subfolder)::dds_::@(spec.base_type.type)_")
struct @(spec.base_type.type)
{

@[if spec.fields]@
@[  for field in spec.fields]@
@{    idl_typedef, idl_typedef_var, idl_type = msg_type_to_idl_2(field.type)}@
@[    if idl_typedef and idl_typedef_var]@
@(      spec.base_type.pkg_name)__@(spec.base_type.type)__@(idl_type) @(field.name);
@[    else]@
  @(idl_type) @(field.name);
@[    end if]@
@[  end for]@
@[else]@
  boolean _dummy;
@[end if]@

};  // struct @(spec.base_type.type)

@[for line in get_post_struct_lines(spec)]@
@(line)
@[end for]@

};  // module dds_

};  // module @(subfolder)

};  // module @(spec.base_type.pkg_name)

#endif  // __@(spec.base_type.pkg_name)__@(subfolder)__@(spec.base_type.type)__idl__
