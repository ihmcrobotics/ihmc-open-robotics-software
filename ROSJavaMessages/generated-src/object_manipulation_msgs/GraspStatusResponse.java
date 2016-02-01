package object_manipulation_msgs;

public interface GraspStatusResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspStatusResponse";
  static final java.lang.String _DEFINITION = "\nbool is_hand_occupied";
  boolean getIsHandOccupied();
  void setIsHandOccupied(boolean value);
}
