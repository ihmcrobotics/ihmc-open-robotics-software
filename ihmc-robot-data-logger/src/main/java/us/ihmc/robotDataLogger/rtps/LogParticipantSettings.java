package us.ihmc.robotDataLogger.rtps;

import javafx.util.Pair;
import us.ihmc.pubsub.attributes.ReliabilityKind;

public class LogParticipantSettings
{
   
   public static final int domain = 7;
   public static final String namespaceSeperator = "/";
   public static final String partition = namespaceSeperator + "us" + namespaceSeperator + "ihmc" + namespaceSeperator + "robotDataLogger";
   
   public static final Pair<String, ReliabilityKind> annoucement = new Pair<>("sessions", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> handshake = new Pair<>("handshake", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> modelFile = new Pair<>("modelFile", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> resourceBundle = new Pair<>("resourceBundle", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> variableChange = new Pair<>("changeVariable", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> clearLog = new Pair<>("clearLog", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> timestamp = new Pair<>("timestamps", ReliabilityKind.BEST_EFFORT);
   public static final Pair<String, ReliabilityKind> data = new Pair<>("data", ReliabilityKind.BEST_EFFORT);
   
   public static final String modelFileTypeName = "us::ihmc::robotDataLogger::modelFile";
   public static final String resourceBundleTypeName = "us::ihmc::robotDataLogger::resourceBundle";
   
   
}
