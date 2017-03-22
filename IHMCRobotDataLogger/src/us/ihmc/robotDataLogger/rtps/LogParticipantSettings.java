package us.ihmc.robotDataLogger.rtps;

public class LogParticipantSettings
{
   public static final int handshakeTimeout = 15000; //15 seconds
   
   public static final int domain = 7;
   public static final String namespaceSeperator = "/";
   public static final String partition = namespaceSeperator + "us" + namespaceSeperator + "ihmc" + namespaceSeperator + "robotDataLogger";
   
   public static final String annoucementTopic = "announce";
   public static final String handshakeTopic = "handshake";
   public static final String modelFileTopic = "modelFile";
   public static final String resourceBundleTopic = "resourceBundle";
   public static final String variableChangeTopic = "changeVariable";
   
   public static final String modelFileTypeName = "us::ihmc::robotDataLogger::modelFile";
   public static final String resourceBundleTypeName = "us::ihmc::robotDataLogger::resourceBundle";
   
   
}
