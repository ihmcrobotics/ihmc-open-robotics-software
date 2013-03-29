package nodelet;

public interface NodeletLoad extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nodelet/NodeletLoad";
  static final java.lang.String _DEFINITION = "string name\nstring type\nstring[] remap_source_args\nstring[] remap_target_args\nstring[] my_argv\n\nstring bond_id\n---\nbool success\n";
}
