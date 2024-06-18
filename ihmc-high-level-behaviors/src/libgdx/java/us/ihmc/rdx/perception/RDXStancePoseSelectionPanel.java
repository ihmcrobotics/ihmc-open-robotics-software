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
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

public class RDXStancePoseSelectionPanel extends RDXPanel implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private ModelInstance pickPointSphere;

   private final ArrayList<ModelInstance> leftSpheres = new ArrayList<>();
   private final ArrayList<ModelInstance> rightSpheres = new ArrayList<>();

   private final SideDependentList<FramePose3D> stancePoses = new SideDependentList<>();
   private final FramePose3D latestPickPoint = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final SideDependentList<RDXFootstepGraphic> footstepGraphics;

   private final StancePoseCalculator stancePoseCalculator;
   private final FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();

   private boolean selectionActive = false;
   private final ImBoolean calculateStancePose = new ImBoolean(false);
   private final RDXStoredPropertySetTuner stancePoseCalculatorParametersTuner = new RDXStoredPropertySetTuner("Stance Pose Parameters");

   private final ROS2Helper ros2Helper;

   public RDXStancePoseSelectionPanel(ROS2Helper ros2Helper, StancePoseCalculator stancePoseCalculator)
   {
      super("Stance Pose Selection");
      setRenderMethod(this::renderImGuiWidgets);
      this.stancePoseCalculator = stancePoseCalculator;

      stancePoseCalculatorParametersTuner.create(stancePoseCalculator.getStancePoseParameters());

      this.ros2Helper = ros2Helper;

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = new SideDependentList<>();
      contactPoints.set(RobotSide.LEFT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));
      contactPoints.set(RobotSide.RIGHT, PlannerTools.createFootContactPoints(0.2, 0.1, 0.08));

      footstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT), new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));

      footstepGraphics.get(RobotSide.LEFT).create();
      footstepGraphics.get(RobotSide.RIGHT).create();

      pickPointSphere = RDXModelBuilder.createSphere(0.04f, Color.CYAN);
   }

   public void update(TerrainMapData terrainMapData, HeightMapData heightMapData)
   {
      environmentHandler.setHeightMap(heightMapData);
      environmentHandler.setTerrainMapData(terrainMapData);
      
      updatePoses();
   }

   private void updatePoses()
   {
      if (environmentHandler.hasTerrainMapData() && environmentHandler.hasHeightMap())
      {
         TerrainMapData terrainMapData = environmentHandler.getTerrainMapData();
         double height = terrainMapData.getHeightInWorld(latestPickPoint.getTranslation().getX32(), latestPickPoint.getTranslation().getY32());

         if (selectionActive)
         {
            latestPickPoint.getTranslation().setZ(height);
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

      LibGDXTools.toLibGDX(latestPickPoint.getPosition(), pickPointSphere.transform);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();
      latestPickPoint.getTranslation().set(pickPointInWorld);

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

   private void setGoalFootsteps()
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(stancePoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(stancePoses.get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      ros2Helper.publish(ContinuousWalkingAPI.PLACED_GOAL_FOOTSTEPS, poseListMessage);
   }

   public void renderImGuiWidgets()
   {

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
      if (ImGui.button("Print Contact Map"))
      {
         PerceptionDebugTools.printMat("Contact Map", terrainMapData.getContactMap(), 4);
      }
      ImGui.sameLine();
      if (ImGui.button("Print Height Map"))
      {
         PerceptionDebugTools.printMat("Height Map", terrainMapData.getHeightMap(), 4);
      }
      ImGui.text("World Point: " + latestPickPoint.getTranslation().toString("%.3f"));
      if (terrainMapData != null && terrainMapData.getHeightMap() != null)
      {
         ImGui.text("Height: " + terrainMapData.getHeightInWorld(latestPickPoint.getTranslation().getX32(), latestPickPoint.getTranslation().getY32()));
         ImGui.text("Contact Score: " + terrainMapData.getContactScoreInWorld(latestPickPoint.getTranslation().getX32(), latestPickPoint.getTranslation().getY32()));
      }

      ImGui.checkbox(labels.get("Calculate Stance Pose"), calculateStancePose);
      if (ImGui.collapsingHeader(labels.get("Stance Pose Parameters")))
      {
         stancePoseCalculatorParametersTuner.renderImGuiWidgets();
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

   public boolean isSelectionActive()
   {
      return selectionActive;
   }
}
