package pr2_msgs;

public interface AccessPoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/AccessPoint";
  static final java.lang.String _DEFINITION = "# This message communicates the state of the PR2\'s wifi access point.\nHeader header\nstring essid\nstring macaddr\nint32 signal\nint32 noise\nint32 snr\nint32 channel\nstring rate\nstring tx_power\nint32 quality\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getEssid();
  void setEssid(java.lang.String value);
  java.lang.String getMacaddr();
  void setMacaddr(java.lang.String value);
  int getSignal();
  void setSignal(int value);
  int getNoise();
  void setNoise(int value);
  int getSnr();
  void setSnr(int value);
  int getChannel();
  void setChannel(int value);
  java.lang.String getRate();
  void setRate(java.lang.String value);
  java.lang.String getTxPower();
  void setTxPower(java.lang.String value);
  int getQuality();
  void setQuality(int value);
}
