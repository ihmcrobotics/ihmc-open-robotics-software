package pr2_msgs;

public interface BatteryState2 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/BatteryState2";
  static final java.lang.String _DEFINITION = "# This message communicates the state of a single battery.\n# Battery Controller Flags, one per battery\nbool present       # is this pack present\nbool charging      # is this pack charging\nbool discharging   # is this pack discharging\nbool power_present # is there an input voltage\nbool power_no_good # is there a fault (No Good)\nbool inhibited     # is this pack disabled for some reason\n# These registers are per battery\ntime      last_battery_update     # last time any battery update occurred\nint16[48] battery_register        # value of this register in the battery\nbool[48]  battery_update_flag     # Has this register ever been updated\ntime[48]  battery_register_update # last time this specific register was updated\n";
  boolean getPresent();
  void setPresent(boolean value);
  boolean getCharging();
  void setCharging(boolean value);
  boolean getDischarging();
  void setDischarging(boolean value);
  boolean getPowerPresent();
  void setPowerPresent(boolean value);
  boolean getPowerNoGood();
  void setPowerNoGood(boolean value);
  boolean getInhibited();
  void setInhibited(boolean value);
  org.ros.message.Time getLastBatteryUpdate();
  void setLastBatteryUpdate(org.ros.message.Time value);
  short[] getBatteryRegister();
  void setBatteryRegister(short[] value);
  boolean[] getBatteryUpdateFlag();
  void setBatteryUpdateFlag(boolean[] value);
  java.util.List<org.ros.message.Time> getBatteryRegisterUpdate();
  void setBatteryRegisterUpdate(java.util.List<org.ros.message.Time> value);
}
