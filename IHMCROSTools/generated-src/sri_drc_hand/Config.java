package sri_drc_hand;

public interface Config extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sri_drc_hand/Config";
  static final java.lang.String _DEFINITION = "uint8 CONFIG_ACT_NONE=0\nuint8 CONFIG_ACT_READ=1\nuint8 CONFIG_ACT_WRITE=2\nuint8 CONFIG_ACT_WRITE_FLASH=3\n\n# send a set of action/key/value tuples to hw.\nuint8[] actions\nuint16[] keys\nfloat32[] values\n";
  static final byte CONFIG_ACT_NONE = 0;
  static final byte CONFIG_ACT_READ = 1;
  static final byte CONFIG_ACT_WRITE = 2;
  static final byte CONFIG_ACT_WRITE_FLASH = 3;
  org.jboss.netty.buffer.ChannelBuffer getActions();
  void setActions(org.jboss.netty.buffer.ChannelBuffer value);
  short[] getKeys();
  void setKeys(short[] value);
  float[] getValues();
  void setValues(float[] value);
}
