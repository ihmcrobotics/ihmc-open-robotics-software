package nodelet;

public interface NodeletLoadRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nodelet/NodeletLoadRequest";
  static final java.lang.String _DEFINITION = "string name\nstring type\nstring[] remap_source_args\nstring[] remap_target_args\nstring[] my_argv\n\nstring bond_id\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getType();
  void setType(java.lang.String value);
  java.util.List<java.lang.String> getRemapSourceArgs();
  void setRemapSourceArgs(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getRemapTargetArgs();
  void setRemapTargetArgs(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getMyArgv();
  void setMyArgv(java.util.List<java.lang.String> value);
  java.lang.String getBondId();
  void setBondId(java.lang.String value);
}
