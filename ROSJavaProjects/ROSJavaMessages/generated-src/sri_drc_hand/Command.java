package sri_drc_hand;

public interface Command extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sri_drc_hand/Command";
  static final java.lang.String _DEFINITION = "# what mode to use for all fingers\nuint8 MODE_IDLE = 0\nuint8 MODE_RESET = 1\nuint8 MODE_CURRENT = 2\nuint8 MODE_FORCE = 3\n";
  static final byte MODE_IDLE = 0;
  static final byte MODE_RESET = 1;
  static final byte MODE_CURRENT = 2;
  static final byte MODE_FORCE = 3;
}
