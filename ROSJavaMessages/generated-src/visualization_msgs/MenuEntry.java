package visualization_msgs;

public interface MenuEntry extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "visualization_msgs/MenuEntry";
  static final java.lang.String _DEFINITION = "# MenuEntry message.\n\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry\'s id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type\n";
  static final byte FEEDBACK = 0;
  static final byte ROSRUN = 1;
  static final byte ROSLAUNCH = 2;
  int getId();
  void setId(int value);
  int getParentId();
  void setParentId(int value);
  java.lang.String getTitle();
  void setTitle(java.lang.String value);
  java.lang.String getCommand();
  void setCommand(java.lang.String value);
  byte getCommandType();
  void setCommandType(byte value);
}
