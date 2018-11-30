package us.ihmc.robotDataLogger.rtps;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.pubsub.attributes.ReliabilityKind;

public class LogParticipantSettings
{
   
   public static final int domain = 7;
   public static final String namespaceSeperator = "/";
   public static final String partition = namespaceSeperator + "us" + namespaceSeperator + "ihmc" + namespaceSeperator + "robotDataLogger";
   
   public static final Pair<String, ReliabilityKind> annoucement = new ImmutablePair<>("sessions", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> handshake = new ImmutablePair<>("handshake", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> modelFile = new ImmutablePair<>("modelFile", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> resourceBundle = new ImmutablePair<>("resourceBundle", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> variableChange = new ImmutablePair<>("changeVariable", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> clearLog = new ImmutablePair<>("clearLog", ReliabilityKind.RELIABLE);
   public static final Pair<String, ReliabilityKind> timestamp = new ImmutablePair<>("timestamps", ReliabilityKind.BEST_EFFORT);
   public static final Pair<String, ReliabilityKind> data = new ImmutablePair<>("data", ReliabilityKind.BEST_EFFORT);
   
   public static final String modelFileTypeName = "us::ihmc::robotDataLogger::modelFile";
   public static final String resourceBundleTypeName = "us::ihmc::robotDataLogger::resourceBundle";
   
   
}
