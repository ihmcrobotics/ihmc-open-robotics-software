package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

public class RDXStancePoseSelectionPanel extends RDXPanel implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private ModelInstance pickPointSphere;

   private ArrayList<ModelInstance> leftSpheres = new ArrayList<>();
   private ArrayList<ModelInstance> rightSpheres = new ArrayList<>();

   public SideDependentList<FramePose3D> stancePoses = new SideDependentList<>();
   private final FramePose3D latestPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final SideDependentList<RDXFootstepGraphic> footstepGraphics;

   private StancePoseCalculator stancePoseCalculator;
   private final FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
   private ImGui3DViewInput latestInput;

   private boolean placed = false;
   private boolean selectionActive = false;
   private ImBoolean calculateStancePose = new ImBoolean(false);

   private ROS2Helper ros2Helper;

   public RDXStancePoseSelectionPanel(ROS2Helper ros2Helper, StancePoseCalculator stancePoseCalculator)
   {
      super("Stance Pose Selection");
      setRenderMethod(this::renderImGuiWidgets);
      this.stancePoseCalculator = stancePoseCalculator;

      this.ros2Helper = ros2Helper;

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = new SideDependentList<>();
      contactPoints.set(RobotSide.LEFT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));
      contactPoints.set(RobotSide.RIGHT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));

      footstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT), new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));

      footstepGraphics.get(RobotSide.LEFT).create();
      footstepGraphics.get(RobotSide.RIGHT).create();

      pickPointSphere = RDXModelBuilder.createSphere(0.04f, Color.CYAN);
   }

   public void update(SideDependentList<FramePose3D> goalPoses, TerrainMapData terrainMapData, HeightMapData heightMapData)
   {
      environmentHandler.setHeightMap(heightMapData);
      environmentHandler.setTerrainMapData(terrainMapData);

      boolean panel3DIsHovered = latestInput != null && latestInput.isWindowHovered();
      if (panel3DIsHovered && ImGui.isKeyPressed('P') && calculateStancePose.get())
      {
         selectionActive = true;
      }
      if (ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         selectionActive = false;
      }
      updatePoses();
   }

   private void updatePoses()
   {
      if (environmentHandler.hasTerrainMapData() && environmentHandler.hasHeightMap())
      {
         TerrainMapData terrainMapData = environmentHandler.getTerrainMapData();
         double height = terrainMapData.getHeightInWorld(latestPose.getTranslation().getX32(), latestPose.getTranslation().getY32());
         double contactScore = terrainMapData.getContactScoreInWorld(latestPose.getTranslation().getX32(), latestPose.getTranslation().getY32());

         if (selectionActive)
         {
            latestPose.getTranslation().setZ(height);
            stancePoses.set(stancePoseCalculator.getStancePoses(latestPose, terrainMapData, environmentHandler));
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
                  leftSpheres.add(RDXModelBuilder.createSphere(0.015f, Color.BLUE));
                  rightSpheres.add(RDXModelBuilder.createSphere(0.015f, Color.RED));
               }
            }

            for (int i = 0; i < leftPoses.size(); i++)
            {
               LibGDXTools.toLibGDX(leftPoses.get(i).getPosition(), leftSpheres.get(i).transform);
               LibGDXTools.toLibGDX(rightPoses.get(i).getPosition(), rightSpheres.get(i).transform);
            }
         }
      }

      LibGDXTools.toLibGDX(latestPose.getPosition(), pickPointSphere.transform);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;

      Point3DReadOnly pickPointInWorld = input.getGroundPickPointInWorld(0.0f);
      latestPose.getTranslation().set(pickPointInWorld);

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
            double latestFootstepYaw = latestPose.getRotation().getYaw();
            latestPose.getOrientation().setYawPitchRoll(latestFootstepYaw + deltaYaw, 0.0, 0.0);
         }
      }

      if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left) && calculateStancePose.get())
      {
         if (!placed)
            setGoalFootsteps();
         selectionActive = false;
      }

      if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         selectionActive = false;
      }
   }

   private void setGoalFootsteps()
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(stancePoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(stancePoses.get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      ros2Helper.publish(ContinuousWalkingAPI.PLACED_GOAL_FOOTSTEPS, poseListMessage);
      placed = true;
   }

   public void renderImGuiWidgets()
   {
      TerrainMapData terrainMapData = environmentHandler.getTerrainMapData();
      if (ImGui.button("Print Contact Map"))
      {
         PerceptionDebugTools.printMat("Contact Map", terrainMapData.getContactMap(), 4);
      }
      ImGui.sameLine();
      if (ImGui.button("Print Height Map"))
      {
         PerceptionDebugTools.printMat("Height Map", terrainMapData.getHeightMap(), 4);
      }
      ImGui.text("World Point: " + latestPose.getTranslation().toString("%.3f"));
      if (terrainMapData != null)
      {
         ImGui.text("Height: " + terrainMapData.getHeightInWorld(latestPose.getTranslation().getX32(), latestPose.getTranslation().getY32()));
         ImGui.text("Contact Score: " + terrainMapData.getContactScoreInWorld(latestPose.getTranslation().getX32(), latestPose.getTranslation().getY32()));
      }

      ImGui.checkbox(labels.get("Calculate Stance Pose"), calculateStancePose);
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

      for (ModelInstance leftSphere : leftSpheres)
      {
         leftSphere = null;
      }

      for (ModelInstance rightSphere : rightSpheres)
      {
         rightSphere = null;
      }
   }

   public boolean isSelectionActive()
   {
      return selectionActive;
   }
}
