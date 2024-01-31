package us.ihmc.footstepPlanning.bodyPath;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

public class AStarBodyPathPlannerVisualizer
{
   public AStarBodyPathPlannerVisualizer()
   {
      //      File file = loadThroughChooser();
//      File file = new File(System.getProperty("user.home") + File.separator + "heightMapDatasets" + File.separator + "stairs.json");

//      File file = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "stairs.json");
//      Pose3D start = new Pose3D(new Point3D(0.5, 0.0, -0.91), new Quaternion());
//      Pose3D goal = new Pose3D(new Point3D(3.3, 0.13, 0.2), new Quaternion());

      File file = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "stairs.json");
      Pose3D start = new Pose3D(new Point3D(2.0, -2.0, -0.91), new Quaternion());
      Pose3D goal = new Pose3D(new Point3D(3.3, 0.13, 0.2), new Quaternion());

//            File file = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "narrow_passage_1.json");
//      Pose3D start = new Pose3D(new Point3D(0.5, 0.0, -0.91), new Quaternion());
//      Pose3D goal = new Pose3D(new Point3D(3.3, 1.0, 0.2), new Quaternion());

//      File file = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "narrow_passage_2.json");
//      Pose3D start = new Pose3D(new Point3D(0.5, 0.0, -0.91), new Quaternion());
//      Pose3D goal = new Pose3D(new Point3D(3.3, 1.0, 0.2), new Quaternion());

      if (file == null)
      {
         return;
      }
      if (!file.exists())
      {
         LogTools.info("It don't exist: " + file.getAbsolutePath());
         return;
      }

      ObjectMapper objectMapper = new ObjectMapper();
      HeightMapMessage heightMapMessage;
      try
      {
         JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
         InputStream inputStream = new FileInputStream(file);
         JsonNode jsonNode = objectMapper.readTree(inputStream);
         heightMapMessage = serializer.deserialize(jsonNode.toString());
         inputStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return;
      }

      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.setGroundVisible(false);

      scs.addStaticLinkGraphics(heightMapGraphics(heightMapData));

      SideDependentList<ConvexPolygon2D> footPolygon = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

      AStarBodyPathPlanner bodyPathPlanner = new AStarBodyPathPlanner(parameters, new AStarBodyPathPlannerParameters(), footPolygon);
      bodyPathPlanner.setHeightMapData(heightMapData);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      for(RobotSide robotSide : RobotSide.values())
      {
         request.getStartFootPoses().get(robotSide).set(start);
         request.getGoalFootPoses().get(robotSide).set(goal);
         request.getStartFootPoses().get(robotSide).appendTranslation(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         request.getGoalFootPoses().get(robotSide).appendTranslation(0.0, robotSide.negateIfRightSide(0.1), 0.0);
      }
      request.setHeightMapData(heightMapData);

      FootstepPlannerOutput output = new FootstepPlannerOutput();

      bodyPathPlanner.handleRequest(request, output);
      List<Pose3D> path = output.getBodyPath();

      if (path != null)
         scs.addStaticLinkGraphics(pathGraphics(path));

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private File loadThroughChooser()
   {
      JFileChooser fileChooser = new JFileChooser();
      fileChooser.setDialogTitle("Import Height Map");
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));
      fileChooser.setCurrentDirectory(new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs"));
      int chooserState = fileChooser.showOpenDialog(null);
      if (chooserState != JFileChooser.APPROVE_OPTION)
         return null;
      return fileChooser.getSelectedFile();
   }

   private static Graphics3DObject heightMapGraphics(HeightMapData heightMapData)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.1);

      double groundHeight = heightMapData.getEstimatedGroundHeight();
      AppearanceDefinition cellColor = YoAppearance.Olive();

      for (int i = 0; i < heightMapData.getNumberOfOccupiedCells(); i++)
      {
         Point2D cellPosition = heightMapData.getCellPosition(i);
         double height = heightMapData.getHeight(i);
         double renderedHeight = (height - groundHeight);

         graphics3DObject.identity();
         graphics3DObject.translate(cellPosition.getX(), cellPosition.getY(), groundHeight + 0.5 * renderedHeight);
         graphics3DObject.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), renderedHeight, true, cellColor);
      }

      graphics3DObject.identity();
      graphics3DObject.translate(heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getEstimatedGroundHeight());
      graphics3DObject.addCube(heightMapData.getGridSizeXY(), heightMapData.getGridSizeXY(), 0.01, YoAppearance.Blue());

      return graphics3DObject;
   }

   private static Graphics3DObject pathGraphics(List<Pose3D> path)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      for (int i = 0; i < path.size(); i++)
      {
         graphics3DObject.identity();

         RigidBodyTransform transform = new RigidBodyTransform();
         path.get(i).get(transform);
         graphics3DObject.transform(transform);
         graphics3DObject.translate(0.0, 0.0, 0.01);
         graphics3DObject.addCoordinateSystem(0.35);
      }

      return graphics3DObject;
   }

   public static void main(String[] args)
   {
      new AStarBodyPathPlannerVisualizer();
   }
}