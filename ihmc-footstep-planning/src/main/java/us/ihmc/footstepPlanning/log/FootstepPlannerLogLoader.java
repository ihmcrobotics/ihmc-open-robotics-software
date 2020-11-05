package us.ihmc.footstepPlanning.log;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoVariableType;

import javax.swing.*;
import java.io.*;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class FootstepPlannerLogLoader
{
   private final JSONSerializer<FootstepPlanningRequestPacket> requestPacketSerializer = new JSONSerializer<>(new FootstepPlanningRequestPacketPubSubType());
   private final JSONSerializer<VisibilityGraphsParametersPacket> bodyPathParametersSerializer = new JSONSerializer<>(new VisibilityGraphsParametersPacketPubSubType());
   private final JSONSerializer<FootstepPlannerParametersPacket> footstepParametersSerializer  = new JSONSerializer<>(new FootstepPlannerParametersPacketPubSubType());
   private final JSONSerializer<SwingPlannerParametersPacket> swingParametersSerializer  = new JSONSerializer<>(new SwingPlannerParametersPacketPubSubType());
   private final JSONSerializer<FootstepPlanningToolboxOutputStatus> statusPacketSerializer = new JSONSerializer<>(new FootstepPlanningToolboxOutputStatusPubSubType());

   private FootstepPlannerLog log = null;
   private final ObjectMapper objectMapper = new ObjectMapper();

   public enum LoadResult
   {
      LOADED, CANCELLED, ERROR
   }

   public enum LoadRequestType
   {
      FILE_CHOOSER, LATEST, NEXT, PREVIOUS
   }

   public LoadResult load(LoadRequestType loadRequestType, FootstepPlannerLog alreadyOpenLog)
   {
      if (loadRequestType == LoadRequestType.FILE_CHOOSER)
      {
         return load();
      }
      else
      {
         if ((loadRequestType == LoadRequestType.PREVIOUS || loadRequestType == LoadRequestType.NEXT) && alreadyOpenLog == null)
         {
            LogTools.warn("Must have log open to view {} log", loadRequestType.name());
            return LoadResult.ERROR;
         }

         SortedSet<Path> sortedLogFolderPaths = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));
         PathTools.walkFlat(Paths.get(FootstepPlannerLogger.defaultLogsDirectory), (path, type) -> {
            if (type == BasicPathVisitor.PathType.DIRECTORY
                && path.getFileName().toString().endsWith(FootstepPlannerLogger.FOOTSTEP_PLANNER_LOG_POSTFIX))
               sortedLogFolderPaths.add(path);
            return FileVisitResult.CONTINUE;
         });

         if (loadRequestType == LoadRequestType.LATEST)
         {
            return load(sortedLogFolderPaths.last().toFile());
         }
         else
         {
            Path loadedLog = sortedLogFolderPaths.last().getParent().resolve(alreadyOpenLog.getLogName());

            if (loadRequestType == LoadRequestType.PREVIOUS)
            {
               SortedSet<Path> headSet = sortedLogFolderPaths.headSet(loadedLog);
               if (headSet.isEmpty())
               {
                  LogTools.warn("No earlier logs found!");
                  return LoadResult.ERROR;
               }
               else
               {
                  return load(headSet.last().toFile());
               }
            }
            else // if (type == LoadRequestType.NEXT)
            {
               SortedSet<Path> tailSet = sortedLogFolderPaths.tailSet(loadedLog);
               tailSet.remove(tailSet.first());
               if (tailSet.isEmpty())
               {
                  LogTools.warn("No newer logs found!");
                  return LoadResult.ERROR;
               }
               else
               {
                  return load(tailSet.first().toFile());
               }
            }
         }
      }
   }

   public LoadResult load()
   {
      JFileChooser fileChooser = new JFileChooser();
      File logDirectory = new File(FootstepPlannerLogger.getDefaultLogsDirectory());
      fileChooser.setCurrentDirectory(logDirectory);
      fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState == JFileChooser.CANCEL_OPTION)
      {
         return LoadResult.CANCELLED;
      }
      else if (chooserState != JFileChooser.APPROVE_OPTION)
      {
         return LoadResult.ERROR;
      }

      return load(fileChooser.getSelectedFile());
   }

   /**
    * Loads the given log file. If successful, retreive the log by calling {@link #getLog}
    * @return if the log loaded successfully
    */
   public LoadResult load(File logDirectory)
   {
      if (!logDirectory.isDirectory())
      {
         LogTools.error("The given file isn't a directory. This method should receive a directory as input");
         return LoadResult.ERROR;
      }

      try
      {
         log = new FootstepPlannerLog(logDirectory.getName());

         // load request packet
         File requestFile = new File(logDirectory, FootstepPlannerLogger.requestPacketFileName);
         InputStream requestPacketInputStream = new FileInputStream(requestFile);
         JsonNode jsonNode = objectMapper.readTree(requestPacketInputStream);
         log.getRequestPacket().set(requestPacketSerializer.deserialize(jsonNode.toString()));
         requestPacketInputStream.close();

         // load body path parameters packet
         File bodyPathParametersFile = new File(logDirectory, FootstepPlannerLogger.bodyPathParametersFileName);
         InputStream bodyPathParametersPacketInputStream = new FileInputStream(bodyPathParametersFile);
         jsonNode = objectMapper.readTree(bodyPathParametersPacketInputStream);
         log.getBodyPathParametersPacket().set(bodyPathParametersSerializer.deserialize(jsonNode.toString()));
         bodyPathParametersPacketInputStream.close();

         // load footstep parameters packet
         File footstepParametersFile = new File(logDirectory, FootstepPlannerLogger.footstepParametersFileName);
         InputStream footstepParametersPacketInputStream = new FileInputStream(footstepParametersFile);
         jsonNode = objectMapper.readTree(footstepParametersPacketInputStream);
         log.getFootstepParametersPacket().set(footstepParametersSerializer.deserialize(jsonNode.toString()));
         footstepParametersPacketInputStream.close();

         // load swing parameters packet
         File swingParametersFile = new File(logDirectory, FootstepPlannerLogger.swingParametersFileName);
         if (swingParametersFile.exists())
         {
            InputStream swingParametersPacketInputStream = new FileInputStream(swingParametersFile);
            jsonNode = objectMapper.readTree(swingParametersPacketInputStream);
            log.getSwingPlannerParametersPacket().set(swingParametersSerializer.deserialize(jsonNode.toString()));
            swingParametersPacketInputStream.close();
         }

         // load status packet
         File statusFile = new File(logDirectory, FootstepPlannerLogger.statusPacketFileName);
         InputStream statusPacketInputStream = new FileInputStream(statusFile);
         jsonNode = objectMapper.readTree(statusPacketInputStream);
         log.getStatusPacket().set(statusPacketSerializer.deserialize(jsonNode.toString()));
         statusPacketInputStream.close();

         // load body path data file
         File bodyPathPlannerFile = new File(logDirectory, FootstepPlannerLogger.bodyPathPlanFileName);
         BufferedReader dataFileReader = new BufferedReader(new FileReader(bodyPathPlannerFile));
         log.getVisibilityGraphHolder().setStartMapId(getIntCSV(true, dataFileReader.readLine())[0]);
         log.getVisibilityGraphHolder().setGoalMapId(getIntCSV(true, dataFileReader.readLine())[0]);
         log.getVisibilityGraphHolder().setInterRegionsMapId(getIntCSV(true, dataFileReader.readLine())[0]);
         loadVisibilityMap(dataFileReader, log.getVisibilityGraphHolder().getStartVisibilityMap());
         loadVisibilityMap(dataFileReader, log.getVisibilityGraphHolder().getGoalVisibilityMap());
         loadVisibilityMap(dataFileReader, log.getVisibilityGraphHolder().getInterRegionsVisibilityMap());

         int numberOfNavigableRegions = getIntCSV(true, dataFileReader.readLine())[0];
         List<PlanarRegion> planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getRequestPacket().getPlanarRegionsListMessage()).getPlanarRegionsAsList();
         for (int i = 0; i < numberOfNavigableRegions; i++)
         {
            VisibilityMapWithNavigableRegion navigableRegion = loadNavigableRegion(dataFileReader, planarRegionsList);
            if (navigableRegion == null)
            {
               LogTools.error("Couldn't find corresponding planar region in visibility graph log");
               break;
            }
            log.getVisibilityGraphHolder().addNavigableRegion(navigableRegion);
         }

         // load header file
         dataFileReader.close();
         File headerFile = new File(logDirectory, FootstepPlannerLogger.headerFileName);
         dataFileReader = new BufferedReader(new FileReader(headerFile));

         List<String[]> enumValues = new ArrayList<>();

         int numberOfEnums = getIntCSV(true, dataFileReader.readLine())[0];
         for (int i = 0; i < numberOfEnums; i++)
         {
            String line = dataFileReader.readLine();
            if (line == null)
            {
               throw new RuntimeException("Reached end of header file before expected");
            }
            enumValues.add(line.split(":")[1].split(","));
         }

         int numberOfVariables = getIntCSV(true, dataFileReader.readLine())[0];
         for (int i = 0; i < numberOfVariables; i++)
         {
            String line = dataFileReader.readLine();
            if (line == null)
            {
               break;
            }

            String[] descriptorStrings = line.replaceAll("\\s+", "").split(",");
            YoVariableType type = YoVariableType.valueOf(descriptorStrings[1]);
            if (type == YoVariableType.ENUM)
            {
               log.getVariableDescriptors().add(new VariableDescriptor(descriptorStrings[0], type, descriptorStrings[2], enumValues.get(Integer.parseInt(descriptorStrings[3]))));
            }
            else
            {
               log.getVariableDescriptors().add(new VariableDescriptor(descriptorStrings[0], type, descriptorStrings[2]));
            }
         }

         log.getFootPolygons().put(RobotSide.LEFT, readPolygon(dataFileReader.readLine()));
         log.getFootPolygons().put(RobotSide.RIGHT, readPolygon(dataFileReader.readLine()));

         // load data file
         dataFileReader.close();
         File dataFile = new File(logDirectory, FootstepPlannerLogger.dataFileName);
         dataFileReader = new BufferedReader(new FileReader(dataFile));

         while (dataFileReader.readLine() != null)
         {
            FootstepPlannerIterationData iterationData = new FootstepPlannerIterationData();
            iterationData.setParentNode(readNode(dataFileReader.readLine()));
            iterationData.setIdealChildNode(readNode(dataFileReader.readLine()));
            int edges = getIntCSV(true, dataFileReader.readLine())[0];
            readSnapData(iterationData.getParentStartSnapData(), dataFileReader);
            readSnapData(iterationData.getParentEndSnapData(), dataFileReader);
            log.getIterationData().add(iterationData);

            for (int i = 0; i < edges; i++)
            {
               // edge marker
               dataFileReader.readLine();

               FootstepPlannerEdgeData edgeData = new FootstepPlannerEdgeData(numberOfVariables);
               edgeData.setParentNode(iterationData.getParentNode());
               edgeData.setChildNode(readNode(dataFileReader.readLine()));
               edgeData.setSolutionEdge(getBooleanCSV(true, dataFileReader.readLine())[0]);
               readSnapData(edgeData.getEndStepSnapData(), dataFileReader);

               long[] longCSV = getLongCSV(true, dataFileReader.readLine());
               for (int j = 0; j < longCSV.length; j++)
               {
                  edgeData.setData(j, longCSV[j]);
               }

               iterationData.getChildNodes().add(edgeData.getChildNode());
               log.getEdgeDataMap().put(new GraphEdge<>(iterationData.getParentNode(), edgeData.getChildNode()), edgeData);
            }
         }

         return LoadResult.LOADED;
      }
      catch (Exception e)
      {
         LogTools.error("Exception while loading log");
         e.printStackTrace();
         return LoadResult.ERROR;
      }
   }

   public FootstepPlannerLog getLog()
   {
      return log;
   }

   private void loadVisibilityMap(BufferedReader dataFileReader, VisibilityMap visibilityMap) throws IOException
   {
      dataFileReader.readLine();

      int numberOfConnections = getIntCSV(true, dataFileReader.readLine())[0];
      for (int i = 0; i < numberOfConnections; i++)
      {
         double[] connectionPointData = getDoubleCSV(false, dataFileReader.readLine());
         Connection connection = new Connection(new ConnectionPoint3D(connectionPointData[0], connectionPointData[1], connectionPointData[2], 0),
                                                new ConnectionPoint3D(connectionPointData[3], connectionPointData[4], connectionPointData[5], 0));
         visibilityMap.addConnection(connection);
      }

      int numberOfVertices = getIntCSV(true, dataFileReader.readLine())[0];
      for (int i = 0; i < numberOfVertices; i++)
      {
         double[] vertexData = getDoubleCSV(false, dataFileReader.readLine());
         visibilityMap.getVertices().add(new ConnectionPoint3D(vertexData[0], vertexData[1], vertexData[2], 0));
      }
   }

   private VisibilityMapWithNavigableRegion loadNavigableRegion(BufferedReader dataFileReader, List<PlanarRegion> planarRegionsList) throws IOException
   {
      dataFileReader.readLine();

      int mapId = getIntCSV(true, dataFileReader.readLine())[0];
      ClusterType clusterType = ClusterType.fromByte((byte) getIntCSV(true, dataFileReader.readLine())[0]);
      ExtrusionSide extrusionSide = ExtrusionSide.fromByte((byte) getIntCSV(true, dataFileReader.readLine())[0]);
      Optional<PlanarRegion> region = planarRegionsList.stream().filter(r -> r.getRegionId() == mapId).findFirst();

      if (!region.isPresent())
      {
         return null;
      }

      Cluster homeRegionCluster = new Cluster(extrusionSide, clusterType);

      int numberOfNaviableExtrusions = getIntCSV(true, dataFileReader.readLine())[0];
      for (int i = 0; i < numberOfNaviableExtrusions; i++)
      {
         homeRegionCluster.getNavigableExtrusionsInLocal().addPoint(readPoint2D(false, dataFileReader.readLine()));
      }
      int numberOfNonNaviableExtrusions = getIntCSV(true, dataFileReader.readLine())[0];
      for (int i = 0; i < numberOfNonNaviableExtrusions; i++)
      {
         homeRegionCluster.getNonNavigableExtrusionsInLocal().addPoint(readPoint2D(false, dataFileReader.readLine()));
      }
      int numberOfPreferredNaviableExtrusions = getIntCSV(true, dataFileReader.readLine())[0];
      for (int i = 0; i < numberOfPreferredNaviableExtrusions; i++)
      {
         int numberOfExtrusions = getIntCSV(true, dataFileReader.readLine())[0];
         ExtrusionHull extrusionHull = new ExtrusionHull();
         for (int j = 0; j < numberOfExtrusions; j++)
         {
            extrusionHull.addPoint(readPoint2D(false, dataFileReader.readLine()));
         }
         homeRegionCluster.getPreferredNavigableExtrusionsInLocal().add(extrusionHull);
      }
      int numberOfPreferredNonNaviableExtrusions = getIntCSV(true, dataFileReader.readLine())[0];
      for (int i = 0; i < numberOfPreferredNonNaviableExtrusions; i++)
      {
         int numberOfExtrusions = getIntCSV(true, dataFileReader.readLine())[0];
         ExtrusionHull extrusionHull = new ExtrusionHull();
         for (int j = 0; j < numberOfExtrusions; j++)
         {
            extrusionHull.addPoint(readPoint2D(false, dataFileReader.readLine()));
         }
         homeRegionCluster.getPreferredNonNavigableExtrusionsInLocal().add(extrusionHull);
      }

      VisibilityMap visibilityMapInLocal = new VisibilityMap();
      loadVisibilityMap(dataFileReader, visibilityMapInLocal);

      VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(new NavigableRegion(region.get(),
                                                                                                                                   homeRegionCluster,
                                                                                                                                   new ArrayList<>()));
      visibilityMapWithNavigableRegion.setVisibilityMapInLocal(visibilityMapInLocal);
      return visibilityMapWithNavigableRegion;
   }

   private static int[] getIntCSV(boolean removeKey, String dataFileLine)
   {
      if(dataFileLine.contains("null"))
         return new int[0];

      String[] csvString = getStringCSV(removeKey, dataFileLine);
      int[] data = new int[csvString.length];
      for (int i = 0; i < csvString.length; i++)
      {
         data[i] = Integer.parseInt(csvString[i]);
      }
      return data;
   }

   private static long[] getLongCSV(boolean removeKey, String dataFileLine)
   {
      if(dataFileLine.contains("null"))
         return new long[0];

      String[] csvString = getStringCSV(removeKey, dataFileLine);
      long[] data = new long[csvString.length];
      for (int i = 0; i < csvString.length; i++)
      {
         data[i] = Long.parseLong(csvString[i]);
      }
      return data;
   }

   private static double[] getDoubleCSV(boolean removeKey, String dataFileLine)
   {
      if(dataFileLine.contains("null"))
         return new double[0];

      String[] csvString = getStringCSV(removeKey, dataFileLine);
      double[] data = new double[csvString.length];
      for (int i = 0; i < csvString.length; i++)
      {
         data[i] = Double.parseDouble(csvString[i]);
      }
      return data;
   }

   private static boolean[] getBooleanCSV(boolean removeKey, String dataFileLine)
   {
      String[] csvString = getStringCSV(removeKey, dataFileLine);
      boolean[] data = new boolean[csvString.length];
      for (int i = 0; i < csvString.length; i++)
      {
         data[i] = Boolean.parseBoolean(csvString[i]);
      }
      return data;
   }

   private static String[] getStringCSV(boolean removeKey, String dataFileLine)
   {
      if(removeKey && !dataFileLine.contains(":"))
      {
         throw new RuntimeException("Error parsing data file, ':' not found at line: \n" + dataFileLine);
      }

      if (removeKey)
      {
         return dataFileLine.split(":")[1].split(",");
      }
      else
      {
         return dataFileLine.split(",");
      }
   }

   private static FootstepGraphNode readNode(String dataFileString)
   {
      int[] csv = getIntCSV(true, dataFileString);
      DiscreteFootstep firstStep = new DiscreteFootstep(csv[0], csv[1], csv[2], RobotSide.values[csv[3]]);
      DiscreteFootstep secondStep = new DiscreteFootstep(csv[4], csv[5], csv[6], RobotSide.values[csv[7]]);
      return new FootstepGraphNode(firstStep, secondStep);
   }

   private void readSnapData(FootstepSnapData footstepSnapDataToPack, BufferedReader dataFileReader) throws IOException
   {
      footstepSnapDataToPack.getSnapTransform().set(readTransform(dataFileReader.readLine()));
      footstepSnapDataToPack.getWiggleTransformInWorld().set(readTransform(dataFileReader.readLine()));
      footstepSnapDataToPack.getCroppedFoothold().set(readPolygon(dataFileReader.readLine()));
      footstepSnapDataToPack.setRegionIndex(getIntCSV(true, dataFileReader.readLine())[0]);
      footstepSnapDataToPack.setAchievedInsideDelta(getDoubleCSV(true, dataFileReader.readLine())[0]);
   }

   private static RigidBodyTransform readTransform(String dataFileLine)
   {
      double[] csv = getDoubleCSV(true, dataFileLine);
      return new RigidBodyTransform(new Quaternion(csv[0], csv[1], csv[2], csv[3]), new Vector3D(csv[4], csv[5], csv[6]));
   }

   private static Point2D readPoint2D(boolean removeKey, String dataFileLine)
   {
      double[] csv = getDoubleCSV(removeKey, dataFileLine);
      return new Point2D(csv[0], csv[1]);
   }

   private static Point3D readPoint3D(boolean removeKey, String dataFileLine)
   {
      double[] csv = getDoubleCSV(removeKey, dataFileLine);
      return new Point3D(csv[0], csv[1], csv[2]);
   }

   private static ConvexPolygon2D readPolygon(String dataFileLine)
   {
      double[] csv = getDoubleCSV(true, dataFileLine);
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      for (int i = 0; i < csv.length / 2; i++)
      {
         polygon.addVertex(csv[2 * i], csv[2 * i + 1]);
      }
      polygon.update();
      return polygon;
   }
}
