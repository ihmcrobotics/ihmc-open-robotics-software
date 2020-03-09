package us.ihmc.footstepPlanning.log;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.*;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.swing.*;
import java.io.*;

public class FootstepPlannerLogLoader
{
   private final JSONSerializer<FootstepPlanningRequestPacket> requestPacketSerializer = new JSONSerializer<>(new FootstepPlanningRequestPacketPubSubType());
   private final JSONSerializer<FootstepPlannerParametersPacket> footstepParametersSerializer  = new JSONSerializer<>(new FootstepPlannerParametersPacketPubSubType());
   private final JSONSerializer<VisibilityGraphsParametersPacket> bodyPathParametersSerializer = new JSONSerializer<>(new VisibilityGraphsParametersPacketPubSubType());
   private final JSONSerializer<FootstepPlanningToolboxOutputStatus> statusPacketSerializer = new JSONSerializer<>(new FootstepPlanningToolboxOutputStatusPubSubType());

   private FootstepPlannerLog log = null;
   private final ObjectMapper objectMapper = new ObjectMapper();

   public boolean load()
   {
      JFileChooser fileChooser = new JFileChooser();
      File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs");
      fileChooser.setCurrentDirectory(logDirectory);
      fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState != JFileChooser.APPROVE_OPTION)
         return false;

      try
      {
         File selectedFile = fileChooser.getSelectedFile();
         log = new FootstepPlannerLog(selectedFile.getName());

         // load request packet
         File requestFile = new File(selectedFile, FootstepPlannerLogger.requestPacketFileName);
         InputStream requestPacketInputStream = new FileInputStream(requestFile);
         JsonNode jsonNode = objectMapper.readTree(requestPacketInputStream);
         log.getRequestPacket().set(requestPacketSerializer.deserialize(jsonNode.toString()));
         requestPacketInputStream.close();

         // load footstep parameters packet
         File footstepParametersFile = new File(selectedFile, FootstepPlannerLogger.footstepParametersFileName);
         InputStream footstepParametersPacketInputStream = new FileInputStream(footstepParametersFile);
         jsonNode = objectMapper.readTree(footstepParametersPacketInputStream);
         log.getFootstepParametersPacket().set(footstepParametersSerializer.deserialize(jsonNode.toString()));
         footstepParametersPacketInputStream.close();

         // load footstep parameters packet
         File bodyPathParametersFile = new File(selectedFile, FootstepPlannerLogger.bodyPathParametersFileName);
         InputStream bodyPathParametersPacketInputStream = new FileInputStream(bodyPathParametersFile);
         jsonNode = objectMapper.readTree(bodyPathParametersPacketInputStream);
         log.getBodyPathParametersPacket().set(bodyPathParametersSerializer.deserialize(jsonNode.toString()));
         bodyPathParametersPacketInputStream.close();

         // load status packet
         File statusFile = new File(selectedFile, FootstepPlannerLogger.statusPacketFileName);
         InputStream statusPacketInputStream = new FileInputStream(statusFile);
         jsonNode = objectMapper.readTree(statusPacketInputStream);
         log.getStatusPacket().set(statusPacketSerializer.deserialize(jsonNode.toString()));
         statusPacketInputStream.close();

         // load data file
         File dataFile = new File(selectedFile, FootstepPlannerLogger.dataFileName);
         BufferedReader dataFileReader = new BufferedReader(new FileReader(dataFile));

         // data variables
         dataFileReader.readLine();

         while(dataFileReader.readLine() != null)
         {
            FootstepPlannerIterationData iterationData = new FootstepPlannerIterationData();
            iterationData.setStanceNode(readNode(dataFileReader.readLine()));
            iterationData.setIdealStep(readNode(dataFileReader.readLine()));
            int edges = getIntCSV(dataFileReader.readLine())[0];
            iterationData.getStanceNodeSnapData().getSnapTransform().set(readTransform(dataFileReader.readLine()));
            iterationData.getStanceNodeSnapData().getCroppedFoothold().set(readPolygon(dataFileReader.readLine()));
            log.getIterationData().add(iterationData);

            for (int i = 0; i < edges; i++)
            {
               // edge marker
               dataFileReader.readLine();

               FootstepPlannerEdgeData edgeData = new FootstepPlannerEdgeData();
               edgeData.setStanceNode(iterationData.getStanceNode());
               edgeData.setCandidateNode(readNode(dataFileReader.readLine()));
               edgeData.getCandidateNodeSnapData().getSnapTransform().set(readTransform(dataFileReader.readLine()));
               edgeData.getCandidateNodeSnapData().getCroppedFoothold().set(readPolygon(dataFileReader.readLine()));

               double[] doubleCSV = getDoubleCSV(dataFileReader.readLine());
               edgeData.setRejectionReason(doubleCSV[0] == -1.0 ? null : BipedalFootstepPlannerNodeRejectionReason.values()[(int) Math.round(doubleCSV[0])]);
               edgeData.setFootAreaPercentage(doubleCSV[1]);
               edgeData.setStepWidth(doubleCSV[2]);
               edgeData.setStepLength(doubleCSV[3]);
               edgeData.setStepHeight(doubleCSV[4]);
               edgeData.setStepReach(doubleCSV[5]);
               edgeData.setCostFromStart(doubleCSV[6]);
               edgeData.setEdgeCost(doubleCSV[7]);
               edgeData.setHeuristicCost(doubleCSV[8]);
               edgeData.setSolutionEdge(doubleCSV[9] > 0.5);
               iterationData.getChildNodes().add(edgeData.getCandidateNode());

               log.getEdgeDataMap().put(new GraphEdge<>(iterationData.getStanceNode(), edgeData.getCandidateNode()), edgeData);
            }
         }

         return true;
      }
      catch (Exception e)
      {
         LogTools.error("Exception while loading log");
         e.printStackTrace();
         return false;
      }
   }

   public FootstepPlannerLog getLog()
   {
      return log;
   }

   private static int[] getIntCSV(String dataFileLine)
   {
      if(dataFileLine.contains("null"))
         return new int[0];

      String[] csvString = getStringCSV(dataFileLine);
      int[] data = new int[csvString.length];
      for (int i = 0; i < csvString.length; i++)
      {
         data[i] = Integer.parseInt(csvString[i]);
      }
      return data;
   }

   private static double[] getDoubleCSV(String dataFileLine)
   {
      if(dataFileLine.contains("null"))
         return new double[0];

      String[] csvString = getStringCSV(dataFileLine);
      double[] data = new double[csvString.length];
      for (int i = 0; i < csvString.length; i++)
      {
         data[i] = Double.parseDouble(csvString[i]);
      }
      return data;
   }

   private static String[] getStringCSV(String dataFileLine)
   {
      if(!dataFileLine.contains(":"))
      {
         throw new RuntimeException("Error parsing data file, ':' not found at line: \n" + dataFileLine);
      }

      return dataFileLine.split(":")[1].split(",");
   }

   private static FootstepNode readNode(String dataFileString)
   {
      int[] csv = getIntCSV(dataFileString);
      return new FootstepNode(csv[0], csv[1], csv[2], RobotSide.values[csv[3]]);
   }

   private static RigidBodyTransform readTransform(String dataFileLine)
   {
      double[] csv = getDoubleCSV(dataFileLine);
      return new RigidBodyTransform(new Quaternion(csv[0], csv[1], csv[2], csv[3]), new Vector3D(csv[4], csv[5], csv[6]));
   }

   private static ConvexPolygon2D readPolygon(String dataFileLine)
   {
      double[] csv = getDoubleCSV(dataFileLine);
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      for (int i = 0; i < csv.length / 2; i++)
      {
         polygon.addVertex(csv[2 * i], csv[2 * i + 1]);
      }
      polygon.update();
      return polygon;
   }

   public static void main(String[] args)
   {
      new FootstepPlannerLogLoader().load();
   }
}
