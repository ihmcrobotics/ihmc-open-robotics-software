package us.ihmc.valkyrie.planner.log;

import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacketPubSubType;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatusPubSubType;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashSet;
import java.util.List;

public class ValkyriePlannerLogger
{
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   static final String requestPacketFileName = "RequestPacket.json";
   static final String statusPacketFileName = "StatusPacket.json";
   static final String dataFileName = "PlannerIterationData.log";

   private final ValkyrieAStarFootstepPlanner planner;
   private String latestLogDirectory;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private FileWriter fileWriter = null;
   private final JSONSerializer<ValkyrieFootstepPlanningRequestPacket> requestPacketSerializer = new JSONSerializer<>(new ValkyrieFootstepPlanningRequestPacketPubSubType());
   private final JSONSerializer<ValkyrieFootstepPlanningStatus> statusPacketSerializer = new JSONSerializer<>(new ValkyrieFootstepPlanningStatusPubSubType());

   public ValkyriePlannerLogger(ValkyrieAStarFootstepPlanner planner)
   {
      this.planner = planner;
   }

   public boolean logSession()
   {
      String sessionDirectory = logDirectory + dateFormat.format(new Date()) + "_" + "ValkyriePlannerLog" + File.separator;
      latestLogDirectory = sessionDirectory;

      // log request packet
      String requestPacketFile = sessionDirectory + "RequestPacket.json";
      try
      {
         FileTools.ensureFileExists(new File(requestPacketFile).toPath());
         outputStream = new FileOutputStream(requestPacketFile);
         printStream = new PrintStream(outputStream);
         printStream.write(requestPacketSerializer.serializeToBytes(planner.getRequestPacket()));
         printStream.flush();
         outputStream.close();
         printStream.close();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging request packet");
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      // log status packet
      String statusPacketFile = sessionDirectory + "StatusPacket.json";
      try
      {
         FileTools.ensureFileExists(new File(statusPacketFile).toPath());
         outputStream = new FileOutputStream(statusPacketFile);
         printStream = new PrintStream(outputStream);
         printStream.write(statusPacketSerializer.serializeToBytes(planner.getPlanningStatus()));
         printStream.flush();
         outputStream.close();
         printStream.close();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging status packet");
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      HashSet<GraphEdge<FootstepNode>> solutionEdges = new HashSet<>();
      List<FootstepNode> solutionPath = planner.getInternalPlanner().getGraph().getPathFromStart(planner.getEndNode());
      for (int i = 0; i < solutionPath.size() - 1; i++)
      {
         solutionEdges.add(new GraphEdge<>(solutionPath.get(i), solutionPath.get(i + 1)));
      }

      // log planner iteration data
      String plannerIterationDataFileName = sessionDirectory + "PlannerIterationData.log";
      try
      {
         File plannerDataFile = new File(plannerIterationDataFileName);
         FileTools.ensureFileExists(plannerDataFile.toPath());
         fileWriter = new FileWriter(plannerIterationDataFileName);

         fileWriter.write("edgeData:" +
                          "rejectionReason, " +
                          "footAreaPercentage, " +
                          "stepWidth, " +
                          "stepLength, " +
                          "stepHeight, " +
                          "stepReach, " +
                          "costFromStart, " +
                          "edgeCost, " +
                          "heuristicCost," +
                          "solutionEdge" + "\n");

         List<ValkyriePlannerIterationData> iterationDataList = planner.getIterationData();
         for (int i = 0; i < iterationDataList.size(); i++)
         {
            ValkyriePlannerIterationData iterationData = iterationDataList.get(i);
            fileWriter.write("-Iteration " + i + "\n");
            writeNode("stanceNode", iterationData.getStanceNode());
            writeNode("idealStep", iterationData.getIdealStep());
            fileWriter.write("edges:" + iterationData.getChildNodes().size() + "\n");
            writeSnapData(iterationData.getStanceNodeSnapData());

            for (int j = 0; j < iterationData.getChildNodes().size(); j++)
            {
               ValkyriePlannerEdgeData edgeData = planner.getEdgeDataMap()
                                                         .get(new GraphEdge<>(iterationData.getStanceNode(), iterationData.getChildNodes().get(j)));

               // indicate start of data
               fileWriter.write("-Edge:\n");
               writeNode("candidateNode", edgeData.getCandidateNode());
               writeSnapData(edgeData.getCandidateNodeSnapData());

               // write additional data as doubles
               boolean solutionEdge = solutionEdges.contains(new GraphEdge<>(iterationData.getStanceNode(), edgeData.getCandidateNode()));
               fileWriter.write("edgeData:" + EuclidCoreIOTools.getStringOf(",",
                                                                             EuclidCoreIOTools.getStringFormat(8, 8),
                                                                             edgeData.getRejectionReason() == null ? -1.0 : (double) edgeData.getRejectionReason().ordinal(),
                                                                             edgeData.getFootAreaPercentage(),
                                                                             edgeData.getStepWidth(),
                                                                             edgeData.getStepLength(),
                                                                             edgeData.getStepHeight(),
                                                                             edgeData.getStepReach(),
                                                                             edgeData.getCostFromStart(),
                                                                             edgeData.getEdgeCost(),
                                                                             edgeData.getHeuristicCost(),
                                                                             solutionEdge ? 1.0 : 0.0));
               fileWriter.write("\n");
            }
         }

         fileWriter.flush();
      }
      catch (Exception e)
      {
         LogTools.error("Error logging edge data");
         outputStream = null;
         printStream = null;
         e.printStackTrace();
         return false;
      }

      return true;
   }

   public String getLatestLogDirectory()
   {
      return latestLogDirectory;
   }

   private void writeNode(String name, FootstepNode node) throws IOException
   {
      if(node == null)
         fileWriter.write(name + ":null" + "\n");
      else
         fileWriter.write(name + ":" + node.getXIndex() + "," + node.getYIndex() + "," + node.getYawIndex() + "," + node.getRobotSide().ordinal() + "\n");
   }

   private void writeSnapData(FootstepNodeSnapData snapData) throws IOException
   {
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      Quaternion quaternion = new Quaternion(snapTransform.getRotation());
      fileWriter.write("snapTransform:" + EuclidCoreIOTools.getStringOf(",",
                                                                         EuclidCoreIOTools.getStringFormat(8, 8),
                                                                         quaternion.getX(),
                                                                         quaternion.getY(),
                                                                         quaternion.getZ(),
                                                                         quaternion.getS(),
                                                                         snapTransform.getTranslation().getX(),
                                                                         snapTransform.getTranslation().getY(),
                                                                         snapTransform.getTranslation().getZ()) + "\n");

      ConvexPolygon2D croppedFoothold = snapData.getCroppedFoothold();
      fileWriter.write("croppedFoothold:");
      if(croppedFoothold.isEmpty() || croppedFoothold.containsNaN())
      {
         fileWriter.write("null");
      }
      else
      {
         for (int vertexIndex = 0; vertexIndex < croppedFoothold.getNumberOfVertices(); vertexIndex++)
         {
            Point2DReadOnly vertex = croppedFoothold.getVertex(vertexIndex);
            fileWriter.write(vertex.getX() + ", " + vertex.getY() + (vertexIndex == croppedFoothold.getNumberOfVertices() - 1 ? "" : ","));
         }
      }

      fileWriter.write("\n");
   }
}
