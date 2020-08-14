package us.ihmc.footstepPlanning.log;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;

import java.io.*;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class FootstepPlannerLogger
{
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
   /** package-private */ static final String defaultLogsDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   /** package-private */ static final String FOOTSTEP_PLANNER_LOG_POSTFIX = "_FootstepPlannerLog";

   static final String requestPacketFileName = "RequestPacket.json";
   static final String bodyPathParametersFileName = "BodyPathParametersPacket.json";
   static final String footstepParametersFileName = "FootstepParametersPacket.json";
   static final String swingParametersFileName = "SwingParametersPacket.json";
   static final String splitFractionParametersFileName = "SplitFractionParametersPacket.json";
   static final String statusPacketFileName = "StatusPacket.json";
   static final String bodyPathPlanFileName = "BodyPathPlanData.log";
   static final String dataFileName = "PlannerIterationData.log";

   private final FootstepPlanningModule planner;
   private final AtomicBoolean generatingLog = new AtomicBoolean();
   private String latestLogDirectory;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private FileWriter fileWriter = null;

   private final FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket();
   private final FootstepPlannerParametersPacket footstepParametersPacket = new FootstepPlannerParametersPacket();
   private final VisibilityGraphsParametersPacket bodyPathParametersPacket = new VisibilityGraphsParametersPacket();
   private final SwingPlannerParametersPacket swingPlannerParametersPacket = new SwingPlannerParametersPacket();
   private final SplitFractionCalculatorParametersPacket splitFractionParametersPacket = new SplitFractionCalculatorParametersPacket();
   private final FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();

   private final JSONSerializer<FootstepPlanningRequestPacket> requestPacketSerializer = new JSONSerializer<>(new FootstepPlanningRequestPacketPubSubType());
   private final JSONSerializer<VisibilityGraphsParametersPacket> bodyPathParametersPacketSerializer = new JSONSerializer<>(new VisibilityGraphsParametersPacketPubSubType());
   private final JSONSerializer<FootstepPlannerParametersPacket> footstepParametersPacketSerializer = new JSONSerializer<>(new FootstepPlannerParametersPacketPubSubType());
   private final JSONSerializer<SwingPlannerParametersPacket> swingPlannerParametersPacketSerializer = new JSONSerializer<>(new SwingPlannerParametersPacketPubSubType());
   private final JSONSerializer<SplitFractionCalculatorParametersPacket> splitFractionParametersPacketSerializer = new JSONSerializer<>(new SplitFractionCalculatorParametersPacketPubSubType());
   private final JSONSerializer<FootstepPlanningToolboxOutputStatus> statusPacketSerializer = new JSONSerializer<>(new FootstepPlanningToolboxOutputStatusPubSubType());

   public FootstepPlannerLogger(FootstepPlanningModule planner)
   {
      this.planner = planner;
   }

   public void logSessionAndReportToMessager(Messager messager)
   {
      if(generatingLog.get())
         return;

      generatingLog.set(true);
      messager.submitMessage(FootstepPlannerMessagerAPI.GenerateLogStatus, "Writing log...");
      boolean success = logSession();

      String status = success ? latestLogDirectory : "Error writing log.";
      messager.submitMessage(FootstepPlannerMessagerAPI.GenerateLogStatus, status);

      generatingLog.set(false);
   }

   public boolean logSession()
   {
      return logSession(defaultLogsDirectory);
   }

   public static void deleteOldLogs(int numberOflogsToKeep)
   {
      deleteOldLogs(numberOflogsToKeep, defaultLogsDirectory);
   }

   public static void deleteOldLogs(int numberOflogsToKeep, String directory)
   {
      SortedSet<Path> sortedSet = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));
      PathTools.walkFlat(Paths.get(directory), (path, type) -> {
         if (type == BasicPathVisitor.PathType.DIRECTORY && path.getFileName().toString().endsWith(FOOTSTEP_PLANNER_LOG_POSTFIX))
            sortedSet.add(path);
         return FileVisitResult.CONTINUE;
      });

      while (sortedSet.size() > numberOflogsToKeep)
      {
         Path earliestLogDirectory = sortedSet.first();
         LogTools.warn("Deleting old log {}", earliestLogDirectory);
         FileTools.deleteQuietly(earliestLogDirectory);
         sortedSet.remove(earliestLogDirectory);
      }
   }

   /**
    * Generates log in the given directory. For example calling with the input "/home/user/.ihmc/logs/" will create (if empty)
    * and populate that directy with log files.
    *
    * <p> Contents of the log file include: json of footstep parameters packet, json of visibility parameters packet, json of request packet,
    * json of terminal output packet, log file containing graph structure and data
    *
    * @return if the logger succeeded
    */
   public boolean logSession(String logDirectory)
   {
      if (!logDirectory.endsWith(File.separator))
      {
         logDirectory += File.separator;
      }

      String sessionDirectory = logDirectory + dateFormat.format(new Date()) + FOOTSTEP_PLANNER_LOG_POSTFIX + File.separator;
      latestLogDirectory = sessionDirectory;

      try
      {
         // log request packet
         String requestPacketFile = sessionDirectory + requestPacketFileName;
         planner.getRequest().setPacket(requestPacket);
         byte[] serializedRequest = requestPacketSerializer.serializeToBytes(requestPacket);
         writeToFile(requestPacketFile, serializedRequest);

         // log body path parameters packet
         String bodyPathParametersPacketFile = sessionDirectory + bodyPathParametersFileName;
         FootstepPlannerMessageTools.copyParametersToPacket(bodyPathParametersPacket, planner.getVisibilityGraphParameters());
         byte[] serializedBodyPathParameters = bodyPathParametersPacketSerializer.serializeToBytes(bodyPathParametersPacket);
         writeToFile(bodyPathParametersPacketFile, serializedBodyPathParameters);

         // log footstep planner parameters packet
         String footstepParametersPacketFile = sessionDirectory + footstepParametersFileName;
         FootstepPlannerMessageTools.copyParametersToPacket(footstepParametersPacket, planner.getFootstepPlannerParameters());
         byte[] serializedFootstepParameters = footstepParametersPacketSerializer.serializeToBytes(footstepParametersPacket);
         writeToFile(footstepParametersPacketFile, serializedFootstepParameters);

         // log swing parameters packet
         String swingParametersPacketFile = sessionDirectory + swingParametersFileName;
         swingPlannerParametersPacket.set(planner.getSwingPlannerParameters().getAsPacket());
         byte[] serializedSwingParameters = swingPlannerParametersPacketSerializer.serializeToBytes(swingPlannerParametersPacket);
         writeToFile(swingParametersPacketFile, serializedSwingParameters);

         // log split fraction parameters packet
         String splitFractionParametersPacketFile = sessionDirectory + splitFractionParametersFileName;
         splitFractionParametersPacket.set(planner.getSplitFractionParameters().getAsPacket());
         byte[] serializedSplitFractionParameters = splitFractionParametersPacketSerializer.serializeToBytes(splitFractionParametersPacket);
         writeToFile(splitFractionParametersPacketFile, serializedSplitFractionParameters);

         // log status packet
         String statusPacketFile = sessionDirectory + statusPacketFileName;
         planner.getOutput().setPacket(outputStatus);
         byte[] serializedStatus = statusPacketSerializer.serializeToBytes(outputStatus);
         writeToFile(statusPacketFile, serializedStatus);
      }
      catch (Exception e)
      {
         LogTools.error("Error generating log");
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      // log body path data
      String bodyPathPlannerDataFileName = sessionDirectory + bodyPathPlanFileName;
      try
      {
         File bodyPathPlannerDataFile = new File(bodyPathPlannerDataFileName);
         FileTools.ensureFileExists(bodyPathPlannerDataFile.toPath());
         fileWriter = new FileWriter(bodyPathPlannerDataFile);

         VisibilityGraphHolder visibilityGraphHolder = planner.getBodyPathPlanner().getVisibilityGraphHolder();
         writeLine(0, "startMapId:" + visibilityGraphHolder.getStartMapId());
         writeLine(0, "goalMapId:" + visibilityGraphHolder.getGoalMapId());
         writeLine(0, "interRegionsMapId:" + visibilityGraphHolder.getInterRegionsMapId());
         writeVisibilityMap("startMap", 0, visibilityGraphHolder.getStartVisibilityMap());
         writeVisibilityMap("goalMap", 0, visibilityGraphHolder.getGoalVisibilityMap());
         writeVisibilityMap("interRegionMap", 0, visibilityGraphHolder.getInterRegionsVisibilityMap());

         int numberOfNavigableRegions = visibilityGraphHolder.getNumberOfNavigableRegions();
         writeLine(0, "navigableRegions:" + numberOfNavigableRegions);
         for (int i = 0; i < numberOfNavigableRegions; i++)
         {
            VisibilityMapWithNavigableRegion navigableRegion = visibilityGraphHolder.getNavigableRegion(i);
            writeNavigableRegion(1, i, navigableRegion);
         }

         fileWriter.flush();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging body path planner data");
         fileWriter = null;
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      // log planner iteration data
      String plannerIterationDataFileName = sessionDirectory + dataFileName;
      try
      {
         File plannerDataFile = new File(plannerIterationDataFileName);
         FileTools.ensureFileExists(plannerDataFile.toPath());
         fileWriter = new FileWriter(plannerIterationDataFileName);

         writeLine(0,
                   "edgeData:" + "rejectionReason, " + "footAreaPercentage, " + "stepWidth, " + "stepLength, " + "stepHeight, " + "stepReach, "
                   + "costFromStart, " + "edgeCost, " + "heuristicCost," + "solutionEdge");

         List<FootstepPlannerIterationData> iterationDataList = planner.getIterationData();
         for (int i = 0; i < iterationDataList.size(); i++)
         {
            FootstepPlannerIterationData iterationData = iterationDataList.get(i);
            fileWriter.write("Iteration " + i + "\n");
            writeNode(1, "stanceNode", iterationData.getStanceNode());
            writeNode(1, "idealStep", iterationData.getIdealStep());
            writeLine(1, "edges:" + iterationData.getChildNodes().size());
            writeSnapData(1, iterationData.getStanceNodeSnapData());

            for (int j = 0; j < iterationData.getChildNodes().size(); j++)
            {
               FootstepPlannerEdgeData edgeData = planner.getEdgeDataMap().get(new GraphEdge<>(iterationData.getStanceNode(), iterationData.getChildNodes().get(j)));

               // indicate start of data
               writeLine(1, "Edge:");
               writeNode(2, "candidateNode", edgeData.getCandidateNode());
               writeSnapData(2, edgeData.getCandidateNodeSnapData());

               // write additional data as doubles
               writeLine(2,
                         "edgeData:" + EuclidCoreIOTools.getStringOf(",",
                                                                     EuclidCoreIOTools.getStringFormat(8, 8),
                                                                     edgeData.getRejectionReason() == null ?
                                                                           -1.0 :
                                                                           (double) edgeData.getRejectionReason().ordinal(),
                                                                     edgeData.getFootAreaPercentage(),
                                                                     edgeData.getStepWidth(),
                                                                     edgeData.getStepLength(),
                                                                     edgeData.getStepHeight(),
                                                                     edgeData.getStepReach(),
                                                                     edgeData.getCostFromStart(),
                                                                     edgeData.getEdgeCost(),
                                                                     edgeData.getHeuristicCost(),
                                                                     edgeData.getSolutionEdge() ? 1.0 : 0.0));
            }
         }

         fileWriter.flush();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging footstep planner data");
         fileWriter = null;
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      return true;
   }

   private void writeToFile(String file, byte[] fileContents) throws Exception
   {
      FileTools.ensureFileExists(new File(file).toPath());
      outputStream = new FileOutputStream(file);
      printStream = new PrintStream(outputStream);

      FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket();
      planner.getRequest().setPacket(requestPacket);
      printStream.write(fileContents);
      printStream.flush();
      outputStream.close();
      printStream.close();
   }

   private void writeNode(int numTabs, String name, FootstepNode node) throws IOException
   {
      if (node == null)
         writeLine(numTabs, name + ":null");
      else
         writeLine(numTabs, name + ":" + node.getXIndex() + "," + node.getYIndex() + "," + node.getYawIndex() + "," + node.getRobotSide().ordinal());
   }

   private void writeSnapData(int numTabs, FootstepNodeSnapData snapData) throws IOException
   {
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      writeTransform(numTabs, "snapTransform: ", new Quaternion(snapTransform.getRotation()), snapTransform.getTranslation());
      RigidBodyTransform wiggleTransform = snapData.getWiggleTransformInWorld();
      writeTransform(numTabs, "wiggleTransform: ", new Quaternion(wiggleTransform.getRotation()), wiggleTransform.getTranslation());

      ConvexPolygon2D croppedFoothold = snapData.getCroppedFoothold();
      if (croppedFoothold.isEmpty() || croppedFoothold.containsNaN())
      {
         writeLine(numTabs, "croppedFoothold: null");
      }
      else
      {
         writeFootPolygon(numTabs, "croppedFoothold:", croppedFoothold);
      }

      int regionIndex = snapData.getRegionIndex();
      writeLine(numTabs, "regionIndex:" + regionIndex);
   }

   private void writeTransform(int numTabs, String name, QuaternionReadOnly orientation, Tuple3DReadOnly translation) throws IOException
   {
      writeLine(numTabs,
                name + EuclidCoreIOTools.getStringOf(",",
                                                     EuclidCoreIOTools.getStringFormat(8, 8),
                                                     orientation.getX(),
                                                     orientation.getY(),
                                                     orientation.getZ(),
                                                     orientation.getS(),
                                                     translation.getX(),
                                                     translation.getY(),
                                                     translation.getZ()));
   }

   private void writeFootPolygon(int numTabs, String name, ConvexPolygon2D croppedFoothold) throws IOException
   {
      for (int i = 0; i < numTabs; i++)
      {
         fileWriter.write(tab);
      }

      fileWriter.write(name);

      for (int vertexIndex = 0; vertexIndex < croppedFoothold.getNumberOfVertices(); vertexIndex++)
      {
         Point2DReadOnly vertex = croppedFoothold.getVertex(vertexIndex);
         fileWriter.write(vertex.getX() + ", " + vertex.getY() + (vertexIndex == croppedFoothold.getNumberOfVertices() - 1 ? "" : ","));
      }

      fileWriter.write("\n");
   }

   private void writePoint2D(int numTabs, Tuple2DReadOnly tuple) throws IOException
   {
      writeLine(numTabs, EuclidCoreIOTools.getStringOf(",", EuclidCoreIOTools.getStringFormat(8, 8), tuple.getX(), tuple.getY()));
   }

   private void writeVisibilityMap(String name, int numTabs, VisibilityMap visibilityMap) throws IOException
   {
      writeLine(numTabs, name);
      writeLine(numTabs + 1, "connections:" + visibilityMap.getConnections().size());
      for(Connection connection : visibilityMap.getConnections())
      {
         ConnectionPoint3D sourcePoint = connection.getSourcePoint();
         ConnectionPoint3D targetPoint = connection.getTargetPoint();
         writeLine(numTabs + 2, EuclidCoreIOTools.getStringOf(",",
                                                                            EuclidCoreIOTools.getStringFormat(8, 8),
                                                                            sourcePoint.getX(),
                                                                            sourcePoint.getY(),
                                                                            sourcePoint.getZ(),
                                                                            targetPoint.getX(),
                                                                            targetPoint.getY(),
                                                                            targetPoint.getZ()));
      }
      writeLine(numTabs + 1, "vertices:" + visibilityMap.getVertices().size());
      for (ConnectionPoint3D vertex : visibilityMap.getVertices())
      {
         writeLine(numTabs + 2, EuclidCoreIOTools.getStringOf(",", EuclidCoreIOTools.getStringFormat(8, 8), vertex.getX(), vertex.getY(), vertex.getZ()));
      }
   }

   private void writeNavigableRegion(int numTabs, int index, VisibilityMapWithNavigableRegion navigableRegion) throws IOException
   {
      writeLine(numTabs, "navigableRegion " + index);
      writeLine(numTabs + 1, "mapId:" + navigableRegion.getMapId());
      writeLine(numTabs + 1, "homeClusterType:" + navigableRegion.getHomeRegionCluster().getType().toByte());
      writeLine(numTabs + 1, "extrusionSide:" + navigableRegion.getHomeRegionCluster().getExtrusionSide().toByte());

      ExtrusionHull navigableExtrusions = navigableRegion.getHomeRegionCluster().getNavigableExtrusionsInLocal();
      writeLine(numTabs + 1, "navigableExtrusions:" + navigableExtrusions.size());
      for (int i = 0; i < navigableExtrusions.size(); i++)
      {
         writePoint2D(numTabs + 2, navigableExtrusions.get(i));
      }

      ExtrusionHull nonNavigableExtrusions = navigableRegion.getHomeRegionCluster().getNonNavigableExtrusionsInLocal();
      writeLine(numTabs + 1, "nonNavigableExtrusions:" + nonNavigableExtrusions.size());
      for (int i = 0; i < nonNavigableExtrusions.size(); i++)
      {
         writePoint2D(numTabs + 2, nonNavigableExtrusions.get(i));
      }

      List<ExtrusionHull> preferredNavigableExtrusions = navigableRegion.getHomeRegionCluster().getPreferredNavigableExtrusionsInLocal();
      writeLine(numTabs + 1, "preferredNavigableExtrusions:" + preferredNavigableExtrusions.size());
      for (int i = 0; i < preferredNavigableExtrusions.size(); i++)
      {
         ExtrusionHull preferredNavigableExtrusion = preferredNavigableExtrusions.get(i);
         writeLine(numTabs + 2, "extrusion:" + preferredNavigableExtrusion.size());
         for (int j = 0; j < preferredNavigableExtrusion.size(); j++)
         {
            writePoint2D(numTabs + 3, preferredNavigableExtrusion.get(j));
         }
      }

      List<ExtrusionHull> preferredNonNavigableExtrusions = navigableRegion.getHomeRegionCluster().getPreferredNonNavigableExtrusionsInLocal();
      writeLine(numTabs + 1, "preferredNonNavigableExtrusions:" + preferredNonNavigableExtrusions.size());
      for (int i = 0; i < preferredNonNavigableExtrusions.size(); i++)
      {
         ExtrusionHull preferredNavigableExtrusion = preferredNonNavigableExtrusions.get(i);
         writeLine(numTabs + 2, "extrusion:" + preferredNavigableExtrusion.size());
         for (int j = 0; j < preferredNavigableExtrusion.size(); j++)
         {
            writePoint2D(numTabs + 3, preferredNavigableExtrusion.get(j));
         }
      }

      writeVisibilityMap("visibilityMapInLocal", numTabs + 1, navigableRegion.getVisibilityMapInLocal());
   }

   private static final String tab = "\t";
   private static final String newLine = "\n";

   private void writeLine(int numTabs, String lineContent) throws IOException
   {
      for (int i = 0; i < numTabs; i++)
      {
         fileWriter.write(tab);
      }

      fileWriter.write(lineContent);
      fileWriter.write(newLine);
   }

   public String getLatestLogDirectory()
   {
      return latestLogDirectory;
   }

   public static String getDefaultLogsDirectory()
   {
      return defaultLogsDirectory;
   }
}
