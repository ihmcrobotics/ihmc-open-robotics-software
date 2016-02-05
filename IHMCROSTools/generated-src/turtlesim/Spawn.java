package turtlesim;

public interface Spawn extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/Spawn";
  static final java.lang.String _DEFINITION = "float32 x\nfloat32 y\nfloat32 theta\nstring name # Optional.  A unique name will be created and returned if this is empty\n---\nstring name";
}
