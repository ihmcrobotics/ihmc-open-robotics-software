package roscpp;

public interface GetLoggersResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "roscpp/GetLoggersResponse";
  static final java.lang.String _DEFINITION = "Logger[] loggers";
  java.util.List<roscpp.Logger> getLoggers();
  void setLoggers(java.util.List<roscpp.Logger> value);
}
