package pr2_msgs;

public interface GPUStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/GPUStatus";
  static final java.lang.String _DEFINITION = "Header header\nstring product_name\nstring pci_device_id\nstring pci_location\nstring display\nstring driver_version\nfloat32 temperature # Temperature in Celcius\nfloat32 fan_speed # Fan speed in rad/s\nfloat32 gpu_usage # Usage in percent\nfloat32 memory_usage # Usage in percent";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getProductName();
  void setProductName(java.lang.String value);
  java.lang.String getPciDeviceId();
  void setPciDeviceId(java.lang.String value);
  java.lang.String getPciLocation();
  void setPciLocation(java.lang.String value);
  java.lang.String getDisplay();
  void setDisplay(java.lang.String value);
  java.lang.String getDriverVersion();
  void setDriverVersion(java.lang.String value);
  float getTemperature();
  void setTemperature(float value);
  float getFanSpeed();
  void setFanSpeed(float value);
  float getGpuUsage();
  void setGpuUsage(float value);
  float getMemoryUsage();
  void setMemoryUsage(float value);
}
