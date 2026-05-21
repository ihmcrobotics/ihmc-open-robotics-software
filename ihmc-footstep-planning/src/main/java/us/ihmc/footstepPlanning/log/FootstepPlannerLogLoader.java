package us.ihmc.footstepPlanning.log;

import perception_msgs.HeightMapMessage;
import toolbox_msgs.*;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.communication.serialization.Ros2MessageCdrFileTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoVariableType;

import javax.swing.*;
import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.stream.Stream;

public class FootstepPlannerLogLoader
{
   private FootstepPlannerLog log = null;

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

         try (Stream<Path> paths = Files.walk(Path.of(FootstepPlannerLogger.defaultLogsDirectory)))
         {
            paths.filter(Files::isDirectory).forEach(dir ->
                                                     {
                                                        if (dir.getFileName().toString().endsWith(FootstepPlannerLogger.FOOTSTEP_PLANNER_LOG_POSTFIX))
                                                        {
                                                           sortedLogFolderPaths.add(dir);
                                                        }
                                                     });
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }

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

      LogTools.info("Loading log: " + logDirectory.getAbsolutePath());

      try
      {
         log = new FootstepPlannerLog(logDirectory.getName());

         // load request packet
         File requestFile = new File(logDirectory, FootstepPlannerLogger.requestPacketFileName);
         loadPacketFromFile(requestFile, log.getRequestPacket());

         // load footstep parameters packet
         File footstepParametersFile = new File(logDirectory, FootstepPlannerLogger.footstepParametersFileName);
         loadPacketFromFile(footstepParametersFile, log.getFootstepParametersPacket());

         // load body path parameters packet
         File bodyPathParametersFile = new File(logDirectory, FootstepPlannerLogger.bodyPathParametersFileName);
         if (bodyPathParametersFile.exists())
         {
            loadPacketFromFile(bodyPathParametersFile, log.getAStarBodyPathPlannerParametersPacket());
         }

         // load swing parameters packet
         File swingParametersFile = new File(logDirectory, FootstepPlannerLogger.swingParametersFileName);
         if (swingParametersFile.exists())
         {
            loadPacketFromFile(swingParametersFile, log.getSwingPlannerParametersPacket());
         }

         // load status packet
         File statusFile = new File(logDirectory, FootstepPlannerLogger.outputStatusPacketFileName);
         loadPacketFromFile(statusFile, log.getStatusPacket());

         // load footstep header file
         File headerFile = new File(logDirectory, FootstepPlannerLogger.headerFileName);
         BufferedReader dataFileReader = new BufferedReader(new FileReader(headerFile));
         int numberOfVariables = loadVariableDescriptors(dataFileReader, log.getVariableDescriptors());
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
            iterationData.setNominalIdealChildNode(readNode(dataFileReader.readLine()));
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

         // load body path header file
         dataFileReader.close();
         headerFile = new File(logDirectory, FootstepPlannerLogger.bodyPathHeaderFileName);
         dataFileReader = new BufferedReader(new FileReader(headerFile));
         numberOfVariables = loadVariableDescriptors(dataFileReader, log.getBodyPathVariableDescriptors());

         // load data file
         dataFileReader.close();
         dataFile = new File(logDirectory, FootstepPlannerLogger.astarBodyPathPlanFileName);
         dataFileReader = new BufferedReader(new FileReader(dataFile));

         while (dataFileReader.readLine() != null)
         {
            AStarBodyPathIterationData iterationData = new AStarBodyPathIterationData();
            iterationData.setParentNode(readBodyPathNode(dataFileReader.readLine()));
            int edges = getIntCSV(true, dataFileReader.readLine())[0];
            iterationData.setParentNodeHeight(getDoubleCSV(true, dataFileReader.readLine())[0]);
            log.getBodyPathIterationData().add(iterationData);

            for (int i = 0; i < edges; i++)
            {
               // edge marker
               dataFileReader.readLine();

               AStarBodyPathEdgeData edgeData = new AStarBodyPathEdgeData(numberOfVariables);
               edgeData.setParentNode(iterationData.getParentNode());
               edgeData.setChildNode(readBodyPathNode(dataFileReader.readLine()));
               edgeData.setSolutionEdge(getBooleanCSV(true, dataFileReader.readLine())[0]);
               edgeData.setChildSnapHeight(getDoubleCSV(true, dataFileReader.readLine())[0]);

               long[] longCSV = getLongCSV(true, dataFileReader.readLine());
               for (int j = 0; j < longCSV.length; j++)
               {
                  edgeData.setData(j, longCSV[j]);
               }

               iterationData.getChildNodes().add(edgeData.getChildNode());
               log.getBodyPathEdgeDataMap().put(new GraphEdge<>(iterationData.getParentNode(), edgeData.getChildNode()), edgeData);
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

   private int loadVariableDescriptors(BufferedReader dataFileReader, List<VariableDescriptor> variableDescriptors) throws IOException
   {
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
            variableDescriptors.add(new VariableDescriptor(descriptorStrings[0], type, descriptorStrings[2], enumValues.get(Integer.parseInt(descriptorStrings[3]))));
         }
         else
         {
            variableDescriptors.add(new VariableDescriptor(descriptorStrings[0], type, descriptorStrings[2]));
         }
      }
      return numberOfVariables;
   }

   public FootstepPlannerLog getLog()
   {
      return log;
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

   private static BodyPathLatticePoint readBodyPathNode(String dataFileString)
   {
      int[] csv = getIntCSV(true, dataFileString);
      return new BodyPathLatticePoint(csv[0], csv[1]);
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

   private static <T extends ROS2Message<T>> void loadPacketFromFile(File file, T message) throws IOException
   {
      try (InputStream inputStream = new FileInputStream(file))
      {
         byte[] bytes = inputStream.readAllBytes();
         if (Ros2MessageCdrFileTools.isLegacyDdsJson(bytes))
         {
            throw new IOException("Legacy JSON footstep planner log at " + file.getAbsolutePath()
                                  + " is not supported after jros2 migration. Re-record the log with the current planner.");
         }
         Ros2MessageCdrFileTools.deserializeInto(bytes, message);
      }
   }

   public static void main(String[] args) throws IOException
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      logLoader.load();

      FootstepPlannerLog log = logLoader.getLog();
      HeightMapMessage heightMapMessage = log.getRequestPacket().getHeightMapMessage();

      byte[] serializedHeightMap = Ros2MessageCdrFileTools.serializeToBytes(heightMapMessage);

      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String fileName = "HeightMap" + dateFormat.format(new Date()) + ".json";
      String file = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + fileName;

      FileTools.ensureFileExists(new File(file).toPath());
      FileOutputStream outputStream = new FileOutputStream(file);
      PrintStream printStream = new PrintStream(outputStream);

      printStream.write(serializedHeightMap);
      printStream.flush();
      outputStream.close();
      printStream.close();

   }
}
