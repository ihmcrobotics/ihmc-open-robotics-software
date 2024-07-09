package us.ihmc.footstepPlanning.log;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import ihmc_common_msgs.msg.dds.StoredPropertySetMessagePubSubType;
import org.apache.commons.lang3.tuple.Pair;
import toolbox_msgs.msg.dds.*;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableType;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

public class FootstepPlannerLogger
{
   private static final SimpleDateFormat directoryDateFormat = new SimpleDateFormat("yyyyMMdd");
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
   private static final int RECOMMENDED_NUMBER_OF_LOGS_TO_KEEP = 500;
   public static final String defaultLogsDirectory;
   static
   {
      String incomingLogsDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator;
      if (ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         incomingLogsDirectory = incomingLogsDirectory + "bamboo-logs" + File.separator
                                                       + System.getenv("bamboo_planKey") + File.separator
                                                       + System.getenv("bamboo_buildResultKey") + File.separator;
      }
      else
      {
         incomingLogsDirectory = incomingLogsDirectory + "logs" + File.separator;
      }
      defaultLogsDirectory = incomingLogsDirectory;
   }
   public static final String FOOTSTEP_PLANNER_LOG_POSTFIX = "_FootstepPlannerLog";

   // File names
   static final String requestPacketFileName = "RequestPacket.json";
   static final String footstepParametersFileName = "FootstepParametersPacket.json";
   static final String bodyPathParametersFileName = "BodyPathParametersPacket.json";
   static final String swingParametersFileName = "SwingParametersPacket.json";
   static final String statusPacketFileName = "StatusPacket.json";
   static final String headerFileName = "Header.txt";
   static final String bodyPathHeaderFileName = "BodyPathHeader.txt";
   static final String dataFileName = "PlannerIterationData.log";
   static final String astarBodyPathPlanFileName = "AStarBodyPathPlanData.log";

   private final FootstepPlanningModule planner;
   private final AtomicBoolean generatingLog = new AtomicBoolean();
   private String latestLogDirectory;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private FileWriter fileWriter = null;

   private final FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket();
   private final FootstepPlannerParametersPacket footstepParametersPacket = new FootstepPlannerParametersPacket();
   private final StoredPropertySetMessage bodyPathParametersPacket = new StoredPropertySetMessage();
   private final SwingPlannerParametersPacket swingPlannerParametersPacket = new SwingPlannerParametersPacket();
   private final FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();

   private final JSONSerializer<FootstepPlanningRequestPacket> requestPacketSerializer = new JSONSerializer<>(new FootstepPlanningRequestPacketPubSubType());
   private final JSONSerializer<FootstepPlannerParametersPacket> footstepParametersPacketSerializer = new JSONSerializer<>(new FootstepPlannerParametersPacketPubSubType());
   private final JSONSerializer<StoredPropertySetMessage> bodyPathParametersPacketSerializer = new JSONSerializer<>(new StoredPropertySetMessagePubSubType());
   private final JSONSerializer<SwingPlannerParametersPacket> swingPlannerParametersPacketSerializer = new JSONSerializer<>(new SwingPlannerParametersPacketPubSubType());
   private final JSONSerializer<FootstepPlanningToolboxOutputStatus> statusPacketSerializer = new JSONSerializer<>(new FootstepPlanningToolboxOutputStatusPubSubType());

   public FootstepPlannerLogger(FootstepPlanningModule planner)
   {
      this.planner = planner;
   }

   /** Keeps around the recommended number of logs. */
   public static void deleteOldLogs()
   {
      deleteOldLogs(RECOMMENDED_NUMBER_OF_LOGS_TO_KEEP, defaultLogsDirectory);
   }

   /** It's recommended to leave quite a few logs around, otherwise, we diminish the usefulness of the logging. */
   public static void deleteOldLogs(int numberOflogsToKeep, String directory)
   {
      SortedSet<Path> sortedLogFolderPaths = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));
      SortedSet<Path> otherDirectoriesInLogsFolder = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));

      try (Stream<Path> paths = Files.walk(Path.of(directory)))
      {
         paths.filter(Files::isDirectory).forEach(dir ->
                                                  {
                                                     if (dir.getFileName().toString().endsWith(FootstepPlannerLogger.FOOTSTEP_PLANNER_LOG_POSTFIX))
                                                     {
                                                        sortedLogFolderPaths.add(dir);
                                                     }
                                                     else
                                                     {
                                                        otherDirectoriesInLogsFolder.add(dir);
                                                     }
                                                  });
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      while (sortedLogFolderPaths.size() > numberOflogsToKeep)
      {
         Path earliestLogDirectory = sortedLogFolderPaths.first();
         LogTools.warn("Deleting old log {}", earliestLogDirectory);
         FileTools.deleteQuietly(earliestLogDirectory);
         sortedLogFolderPaths.remove(earliestLogDirectory);
      }

      // Remove any empty directories inside the logs folder
      for (Path path : otherDirectoriesInLogsFolder)
      {
         try
         {
            if (isDirectoryEmpty(path))
            {
               Files.delete(path);
            }
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   private static boolean isDirectoryEmpty(Path dir) throws IOException
   {
      try (var dirStream = Files.list(dir))
      {
         return dirStream.findAny().isEmpty();
      }
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
      String logDirectoryWithDate = logDirectory + directoryDateFormat.format(new Date());
      String sessionDirectory = generateALogFolderName(logDirectoryWithDate);
      return logSessionWithExactFolderName(sessionDirectory);
   }

   public boolean logSessionWithExactFolderName(String sessionDirectory)
   {
      LogTools.info("Logging Session: " + sessionDirectory);
      latestLogDirectory = sessionDirectory;

      try
      {
         // log request packet
         String requestPacketFile = sessionDirectory + requestPacketFileName;
         planner.getRequest().setPacket(requestPacket);
         byte[] serializedRequest = requestPacketSerializer.serializeToBytes(requestPacket);
         writeToFile(requestPacketFile, serializedRequest);

         // log footstep planner parameters packet
         String footstepParametersPacketFile = sessionDirectory + footstepParametersFileName;
         FootstepPlannerMessageTools.copyParametersToPacket(footstepParametersPacket, planner.getFootstepPlannerParameters());
         byte[] serializedFootstepParameters = footstepParametersPacketSerializer.serializeToBytes(footstepParametersPacket);
         writeToFile(footstepParametersPacketFile, serializedFootstepParameters);

         // log body path planner parameters packet
         String bodyPathParametersPacketFile = sessionDirectory + bodyPathParametersFileName;
         planner.getAStarBodyPathPlannerParameters().getAllAsStrings().forEach(value -> bodyPathParametersPacket.getStrings().add(value));
         byte[] serializedBodyPathParameters = bodyPathParametersPacketSerializer.serializeToBytes(bodyPathParametersPacket);
         writeToFile(bodyPathParametersPacketFile, serializedBodyPathParameters);

         // log swing parameters packet
         String swingParametersPacketFile = sessionDirectory + swingParametersFileName;
         swingPlannerParametersPacket.set(planner.getSwingPlannerParameters().getAsPacket());
         byte[] serializedSwingParameters = swingPlannerParametersPacketSerializer.serializeToBytes(swingPlannerParametersPacket);
         writeToFile(swingParametersPacketFile, serializedSwingParameters);

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
      
      // log planner iteration header file
      try
      {
         File plannerHeaderFile = new File(sessionDirectory + headerFileName);
         FileTools.ensureFileExists(plannerHeaderFile.toPath());
         fileWriter = new FileWriter(plannerHeaderFile);

         logVariableDescriptors(planner.getAStarPlannerRegistry());
         SideDependentList<ConvexPolygon2D> footPolygons = planner.getFootPolygons();
         for (RobotSide robotSide : RobotSide.values)
         {
            writeFootPolygon(0, robotSide.getLowerCaseName() + "FootPolygon:", footPolygons.get(robotSide));
         }

         fileWriter.flush();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging header file");
         fileWriter = null;
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      // log body path planner iteration header file
      try
      {
         File plannerHeaderFile = new File(sessionDirectory + bodyPathHeaderFileName);
         FileTools.ensureFileExists(plannerHeaderFile.toPath());
         fileWriter = new FileWriter(plannerHeaderFile);

         logVariableDescriptors(planner.getBodyPathPlannerRegistry());
         fileWriter.flush();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging header file");
         fileWriter = null;
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      // Log footstep planner iteration data
      String plannerIterationDataFileName = sessionDirectory + dataFileName;
      try
      {
         File plannerDataFile = new File(plannerIterationDataFileName);
         FileTools.ensureFileExists(plannerDataFile.toPath());
         fileWriter = new FileWriter(plannerIterationDataFileName);

         List<FootstepPlannerIterationData> iterationDataList = planner.getIterationData();
         for (int i = 0; i < iterationDataList.size(); i++)
         {
            FootstepPlannerIterationData iterationData = iterationDataList.get(i);
            fileWriter.write("Iteration " + i + newLine);
            writeNode(1, "parentNode", iterationData.getParentNode());
            writeNode(1, "idealStep", iterationData.getIdealChildNode());
            writeNode(1, "nominalIdealStep", iterationData.getNominalIdealChildNode());
            writeLine(1, "edges:" + iterationData.getChildNodes().size());
            writeSnapData(1, iterationData.getParentStartSnapData());
            writeSnapData(1, iterationData.getParentEndSnapData());

            for (int j = 0; j < iterationData.getChildNodes().size(); j++)
            {
               FootstepPlannerEdgeData edgeData = planner.getEdgeDataMap().get(new GraphEdge<>(iterationData.getParentNode(), iterationData.getChildNodes().get(j)));
               if (edgeData != null) // Sometimes it's not there. Not sure if that's expected. TODO: Verify. @dcalvert
               {
                  // indicate start of data
                  writeLine(1, "Edge:");
                  writeNode(2, "candidateNode", edgeData.getChildNode());
                  writeLine(2, "solutionEdge:" + edgeData.isSolutionEdge());
                  writeSnapData(2, edgeData.getEndStepSnapData());

                  // write additional data as doubles
                  fileWriter.write(tab + tab + "data:");
                  long[] dataBuffer = edgeData.getDataBuffer();
                  for (int k = 0; k < dataBuffer.length; k++)
                  {
                     fileWriter.write(dataBuffer[k] + (k == dataBuffer.length - 1 ? "" : ","));
                  }
                  fileWriter.write(newLine);
               }
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

      // Log body path planner iteration data
      try
      {
         File plannerDataFile = new File(sessionDirectory + astarBodyPathPlanFileName);
         FileTools.ensureFileExists(plannerDataFile.toPath());
         fileWriter = new FileWriter(sessionDirectory + astarBodyPathPlanFileName);

         List<AStarBodyPathIterationData> iterationDataList = planner.getBodyPathIterationData();
         for (int i = 0; i < iterationDataList.size(); i++)
         {
            AStarBodyPathIterationData iterationData = iterationDataList.get(i);
            fileWriter.write("Iteration " + i + newLine);
            writeNode(1, "parentNode", iterationData.getParentNode());
            writeLine(1, "edges:" + iterationData.getChildNodes().size());
            writeLine(1, "height:" + iterationData.getParentNodeHeight());

            for (int j = 0; j < iterationData.getChildNodes().size(); j++)
            {
               AStarBodyPathEdgeData edgeData = planner.getBodyPathEdgeDataMap().get(new GraphEdge<>(iterationData.getParentNode(), iterationData.getChildNodes().get(j)));
               if (edgeData == null)
               {
                  System.out.println();
                  throw new RuntimeException("No edge data!" + "\n Parent: " + iterationData.getParentNode() + "\n Child: " + iterationData.getChildNodes().get(j));
               }

               // indicate start of data
               writeLine(1, "Edge:");
               writeNode(2, "candidateNode", edgeData.getChildNode());
               writeLine(2, "solutionEdge:" + edgeData.isSolutionEdge());
               writeLine(2, "height:" + edgeData.getChildSnapHeight());

               // write additional data as doubles
               fileWriter.write(tab + tab + "data:");
               long[] dataBuffer = edgeData.getDataBuffer();
               for (int k = 0; k < dataBuffer.length; k++)
               {
                  fileWriter.write(dataBuffer[k] + (k == dataBuffer.length - 1 ? "" : ","));
               }
               fileWriter.write(newLine);
            }
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

      return true;
   }

   private void logVariableDescriptors(YoRegistry registry) throws IOException
   {
      Map<Class<?>, Integer> enumIndexMap = new HashMap<>();
      List<YoVariable> allVariables = registry.collectSubtreeVariables();

      List<Pair<Class<?>, Enum<?>[]>> enumDescriptions = new ArrayList<>();

      for (int i = 0; i < allVariables.size(); i++)
      {
         YoVariable yoVariable = allVariables.get(i);
         if (yoVariable.getType() == YoVariableType.ENUM)
         {
            YoEnum<?> yoEnum = (YoEnum<?>) yoVariable;
            Class<?> enumType = yoEnum.getEnumType();
            Enum<?>[] enumValues = yoEnum.getEnumValues();

            if (!enumIndexMap.containsKey(enumType))
            {
               enumIndexMap.put(enumType, enumDescriptions.size());
               enumDescriptions.add(Pair.of(enumType, enumValues));
            }
         }
      }

      fileWriter.write("enums:" + enumDescriptions.size() + newLine);
      for (int i = 0; i < enumDescriptions.size(); i++)
      {
         fileWriter.write(tab);
         fileWriter.write(enumDescriptions.get(i).getKey().getSimpleName() + ":");

         Enum<?>[] enumValues = enumDescriptions.get(i).getRight();
         for (int j = 0; j < enumValues.length; j++)
         {
            fileWriter.write(enumValues[j] + (j == enumValues.length - 1 ? "" : ","));
         }

         fileWriter.write(newLine);
      }

      fileWriter.write("variables:" + allVariables.size() + newLine);
      for (int i = 0; i < allVariables.size(); i++)
      {
         YoVariable yoVariable = allVariables.get(i);
         String name = yoVariable.getName();
         YoVariableType type = yoVariable.getType();
         String registryName = yoVariable.getRegistry().getName();

         fileWriter.write(tab);
         fileWriter.write(name + ",");
         fileWriter.write(type + ",");
         fileWriter.write(registryName);

         if (type == YoVariableType.ENUM)
         {
            YoEnum<?> yoEnum = (YoEnum<?>) yoVariable;
            Class<?> enumType = yoEnum.getEnumType();
            fileWriter.write("," + enumIndexMap.get(enumType));
            fileWriter.write( "," + yoEnum.isNullAllowed());
         }

         fileWriter.write(newLine);
      }
   }

   /**
    * This method writes the contents of a file to the filePath specified.
    * This method is intended to be optimized to run as fast as possible
    */
   private void writeToFile(String file, byte[] fileContents) throws Exception
   {
      Path filePath = Paths.get(file);
      FileTools.ensureFileExists(filePath);

      // This uses a try-with-resources to make sure the BufferedOutputStream is closed properly
      try (BufferedOutputStream bufferedOutputStream = new BufferedOutputStream(new FileOutputStream(filePath.toFile())))
      {
         bufferedOutputStream.write(fileContents);
         bufferedOutputStream.flush();
      }
      catch (IOException e)
      {
         System.err.println("Error writing to file: " + e.getMessage());
         e.printStackTrace();
      }
   }

   private void writeNode(int numTabs, String name, BodyPathLatticePoint node) throws IOException
   {
      if (node == null)
      {
         writeLine(numTabs, name + ":null");
      }
      else
      {
         writeLine(numTabs, name + ":" +
                            node.getXIndex() + "," +
                            node.getYIndex());
      }
   }

   private void writeNode(int numTabs, String name, FootstepGraphNode node) throws IOException
   {
      if (node == null)
      {
         writeLine(numTabs, name + ":null");
      }
      else
      {
         DiscreteFootstep firstStep = node.getFirstStep();
         DiscreteFootstep secondStep = node.getSecondStep();
         writeLine(numTabs, name + ":" +
                   firstStep.getXIndex() + "," +
                   firstStep.getYIndex() + "," +
                   firstStep.getYawIndex() + "," +
                   firstStep.getRobotSide().ordinal() + "," +
                   secondStep.getXIndex() + "," +
                   secondStep.getYIndex() + "," +
                   secondStep.getYawIndex() + "," +
                   secondStep.getRobotSide().ordinal());
      }
   }

   private void writeSnapData(int numTabs, FootstepSnapData snapData) throws IOException
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
      writeLine(numTabs, "deltaInside:" + snapData.getAchievedInsideDelta());
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

   private void writeFootPolygon(int numTabs, String name, ConvexPolygon2D footPolygon) throws IOException
   {
      for (int i = 0; i < numTabs; i++)
      {
         fileWriter.write(tab);
      }

      fileWriter.write(name);

      for (int vertexIndex = 0; vertexIndex < footPolygon.getNumberOfVertices(); vertexIndex++)
      {
         Point2DReadOnly vertex = footPolygon.getVertex(vertexIndex);
         fileWriter.write(vertex.getX() + "," + vertex.getY() + (vertexIndex == footPolygon.getNumberOfVertices() - 1 ? newLine : ","));
      }
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

   public static String generateALogFolderName()
   {
      return generateALogFolderName(defaultLogsDirectory);
   }

   public static String generateALogFolderName(String logDirectory)
   {
      if (!logDirectory.endsWith(File.separator))
      {
         logDirectory += File.separator;
      }

      return logDirectory + dateFormat.format(new Date()) + FOOTSTEP_PLANNER_LOG_POSTFIX + File.separator;
   }
}
