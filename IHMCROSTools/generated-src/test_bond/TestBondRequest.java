package test_bond;

public interface TestBondRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_bond/TestBondRequest";
  static final java.lang.String _DEFINITION = "string topic\nstring id\nduration delay_connect\nduration delay_death\nbool inhibit_death\nbool inhibit_death_message\n";
  java.lang.String getTopic();
  void setTopic(java.lang.String value);
  java.lang.String getId();
  void setId(java.lang.String value);
  org.ros.message.Duration getDelayConnect();
  void setDelayConnect(org.ros.message.Duration value);
  org.ros.message.Duration getDelayDeath();
  void setDelayDeath(org.ros.message.Duration value);
  boolean getInhibitDeath();
  void setInhibitDeath(boolean value);
  boolean getInhibitDeathMessage();
  void setInhibitDeathMessage(boolean value);
}
