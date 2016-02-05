package pr2_mechanism_msgs;

public interface ListControllers extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/ListControllers";
  static final java.lang.String _DEFINITION = "# The ListControllers service returns a list of controller names that are spawned\n# inside pr2_mechanism_control, and their corresponding stats. The state is either\n# running or stopped.\n\n---\nstring[] controllers\nstring[] state";
}
