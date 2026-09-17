package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.PoseListMessage;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import perception_msgs.TerrainMapMessage;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

public class RDXStancePoseSelectionPanel extends RDXPanel implements RenderableProvider
{
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2Publisher<PoseListMessage> publisher;
   private final ROS2Publisher<PoseListMessage> turningPublisher;

   private ModelInstance pickPointSphere;

   private final ArrayList<ModelInstance> leftSpheres = new ArrayList<>();
   private final ArrayList<ModelInstance> rightSpheres = new ArrayList<>();

   private final SideDependentList<FramePose3D> stancePoses = new SideDependentList<>();
   private final FramePose3D latestPickPoint = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final SideDependentList<RDXFootstepGraphic> footstepGraphics;

   private final StancePoseCalculator stancePoseCalculator;
   private final EnvironmentHandler environmentHandler = new EnvironmentHandler();

   private boolean selectionActive = false; // Important for determining when to detect collisions or not
   private final ImBoolean calculateStancePose = new ImBoolean(false);
   private final RDXStoredPropertySetTuner stancePoseCalculatorParametersTuner = new RDXStoredPropertySetTuner("Stance Pose Parameters");

   public RDXStancePoseSelectionPanel(RDXBaseUI baseUI, ROS2Node ros2Node, StancePoseCalculator stancePoseCalculator)
   {
      super("Stance Pose Selection");
      this.baseUI = baseUI;
      setRenderMethod(this::renderImGuiWidgets);
      this.stancePoseCalculator = stancePoseCalculator;

      stancePoseCalculatorParametersTuner.create(stancePoseCalculator.getStancePoseParameters());

      publisher = ros2Node.createPublisher(ContinuousHikingAPI.PLACED_GOAL_FOOTSTEPS);
      turningPublisher = ros2Node.createPublisher(ContinuousHikingAPI.ROTATE_GOAL_FOOTSTEPS);

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = new SideDependentList<>();
      contactPoints.set(RobotSide.LEFT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));
      contactPoints.set(RobotSide.RIGHT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));

      footstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT), new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));

      footstepGraphics.get(RobotSide.LEFT).create();
      footstepGraphics.get(RobotSide.RIGHT).create();

      pickPointSphere = RDXModelBuilder.createSphere(0.04f, Color.CYAN);
   }

   public void update(TerrainMapData latestTerrainMapData)
   {
      environmentHandler.setTerrainMapData(latestTerrainMapData);

      if (environmentHandler.hasTerrainMapData())
      {
         TerrainMapData terrainMapData = environmentHandler.getTerrainMapData();

         if (selectionActive)
         {
            stancePoses.set(stancePoseCalculator.getStancePoses(latestPickPoint, terrainMapData, environmentHandler));
            for (RobotSide robotSide : RobotSide.values)
            {
               footstepGraphics.get(robotSide).setPose(stancePoses.get(robotSide));
            }

            ArrayList<FramePose3D> leftPoses = stancePoseCalculator.getLeftPoses();
            ArrayList<FramePose3D> rightPoses = stancePoseCalculator.getRightPoses();

            if (leftSpheres.isEmpty())
            {
               for (int i = 0; i < leftPoses.size(); i++)
               {
                  leftSpheres.add(RDXModelBuilder.createSphere(0.01f, Color.WHITE));
                  rightSpheres.add(RDXModelBuilder.createSphere(0.01f, Color.BLACK));
               }
            }

            for (int i = 0; i < leftPoses.size(); i++)
            {
               LibGDXTools.toLibGDX(leftPoses.get(i).getPosition(), leftSpheres.get(i).transform);
               LibGDXTools.toLibGDX(rightPoses.get(i).getPosition(), rightSpheres.get(i).transform);
            }
         }
      }
      else
      {
         if (selectionActive)
         {
            stancePoses.set(new SideDependentList<>(new FramePose3D(), new FramePose3D()));
            for (RobotSide robotSide : RobotSide.values)
            {
               FramePose3D pose = new FramePose3D(latestPickPoint);
               pose.appendTranslation(0, robotSide == RobotSide.LEFT ? 0.12 : -0.12, 0);
               stancePoses.get(robotSide).set(pose);
               footstepGraphics.get(robotSide).setPose(stancePoses.get(robotSide));
            }
         }
      }

      // NOTE: This is very important for making sure that the collision with the height map is correct
      if (selectionActive)
      {
         baseUI.setModelSceneMouseCollisionEnabled(true);
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Save Height Map To File"))
      {
         saveHeightMapToFile(environmentHandler.getTerrainMapData());
      }

      // Allow for visualizing the stance pose grid
      if (calculateStancePose.get() && ImGui.isKeyPressed('P'))
      {
         selectionActive = true;
      }
      if (ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         selectionActive = false;
      }

      TerrainMapData terrainMapData = environmentHandler.getTerrainMapData();
      ImGui.text("World Point: " + latestPickPoint.getTranslation().toString("%.3f"));
      if (terrainMapData != null)
      {
         ImGui.text("Height: " + terrainMapData.getHeight(latestPickPoint.getTranslation().getX32(), latestPickPoint.getTranslation().getY32()));
         ImGui.text(
               "Traversability Score: " + terrainMapData.getTraversabilityScore(latestPickPoint.getTranslation().getX32(), latestPickPoint.getTranslation().getY32()));
      }

      ImGui.checkbox(labels.get("Calculate Stance Pose"), calculateStancePose);
      if (ImGui.collapsingHeader(labels.get("Stance Pose Parameters")))
      {
         stancePoseCalculatorParametersTuner.renderImGuiWidgets();
      }
   }

   /**
    * This pick point only gets put on the height map correct if the collisions are correct. The UI has to know that we want to account for mouse collisions
    * That happens with {@link RDXBaseUI#setModelSceneMouseCollisionEnabled(boolean)}
    */
   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      if (input == null)
         return;

      Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();
      latestPickPoint.getTranslation().set(pickPointInWorld);
      LibGDXTools.toLibGDX(latestPickPoint.getPosition(), pickPointSphere.transform);

      double deltaYaw = 0.0;
      boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
      if (ctrlHeld)
      {
         float dScroll = input.getMouseWheelDelta();
         if (dScroll > 0.0)
         {
            deltaYaw = 0.03 * Math.PI;
         }
         else if (dScroll < 0.0)
         {
            deltaYaw = -0.03 * Math.PI;
         }
         if (deltaYaw != 0.0)
         {
            double latestFootstepYaw = latestPickPoint.getRotation().getYaw();
            latestPickPoint.getOrientation().setYawPitchRoll(latestFootstepYaw + deltaYaw, 0.0, 0.0);
         }
      }
      if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Middle) && calculateStancePose.get() && selectionActive)
      {
         setRotateGoalFootsteps();
         selectionActive = false;
      }

      if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left) && calculateStancePose.get() && selectionActive)
      {
         setGoalFootsteps();
         selectionActive = false;
      }

      if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         selectionActive = false;
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selectionActive)
      {
         footstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         footstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);

         pickPointSphere.getRenderables(renderables, pool);

         for (ModelInstance leftSphere : leftSpheres)
         {
            leftSphere.getRenderables(renderables, pool);
         }

         for (ModelInstance rightSphere : rightSpheres)
         {
            rightSphere.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      footstepGraphics.get(RobotSide.LEFT).destroy();
      footstepGraphics.get(RobotSide.RIGHT).destroy();
      pickPointSphere = null;
      leftSpheres.clear();
      rightSpheres.clear();
   }

   public void saveHeightMapToFile(TerrainMapData terrainMapData)
   {
      TerrainMapMessage terrainMapMessage = new TerrainMapMessage();
      TerrainMapMessageTools.toMessage(terrainMapData, terrainMapMessage);

      String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS"));
      Path heightMapDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY;

      try
      {
         if (!Files.exists(heightMapDirectory))
         {
            Files.createDirectory(heightMapDirectory);
         }

         String heightMapDirectoryString = heightMapDirectory.toString();
         File heightMapFile = new File(heightMapDirectoryString + "/" + timestamp + "_HeightMapData.json");
         Files.writeString(heightMapFile.toPath(), ROS2MessageCdrFileTools.serializeToBase64(terrainMapMessage));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void setRotateGoalFootsteps()
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(stancePoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(stancePoses.get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      turningPublisher.publish(poseListMessage);
   }

   private void setGoalFootsteps()
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(stancePoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(stancePoses.get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      publisher.publish(poseListMessage);
   }

   public boolean isSelectionActive()
   {
      return selectionActive;
   }
}
