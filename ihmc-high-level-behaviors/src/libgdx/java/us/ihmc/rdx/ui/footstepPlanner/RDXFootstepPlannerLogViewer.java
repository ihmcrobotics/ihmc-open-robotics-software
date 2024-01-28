package us.ihmc.rdx.ui.footstepPlanner;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.rdx.visualizers.RDXSphereAndArrowGraphic;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.File;
import java.util.Set;

public class RDXFootstepPlannerLogViewer
{
   private final RDX3DScene scene3D;
   private final RDX3DPanel panel3D;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiDirectory directory;
   private final TypedNotification<FootstepPlannerLog> logLoadedNotification = new TypedNotification<>();
   private FootstepPlannerLog footstepPlannerLog = null;
   private final SideDependentList<RDXFootstepGraphic> startFootPoses = new SideDependentList<>();
   private final SideDependentList<RDXFootstepGraphic> goalFootPoses = new SideDependentList<>();
   private final RDXSphereAndArrowGraphic goalGraphic;
   private final Pose3D goalPose = new Pose3D();
   private final RDXPlanarRegionsGraphic planarRegionsGraphic;
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private FootstepPlan footstepPlan;
   private FootstepPlannerRejectionReasonReport rejectionReasonReport;
   private final ImBoolean probeMode = new ImBoolean(false);
   private final RDXModelInstance probeSphere;
   private RDX3DPanelTooltip tooltip;
   private final SideDependentList<MutableReferenceFrame> probedFootFrames
         = new SideDependentList<>(new MutableReferenceFrame(ReferenceFrame.getWorldFrame()),
                                   new MutableReferenceFrame(ReferenceFrame.getWorldFrame()));
   private final FramePose3D probePose = new FramePose3D();

   public RDXFootstepPlannerLogViewer(RDXBaseUI baseUI, DRCRobotModel robotModel)
   {
      scene3D = new RDX3DScene();
      scene3D.create(RDXSceneLevel.values());
      scene3D.addDefaultLighting();
      panel3D = new RDX3DPanel("Footstep Planner Log 3D View");
      baseUI.add3DPanel(panel3D, scene3D);
      panel3D.addChild(new RDXPanel("Footstep Planner Log Viewer Controls", this::renderImGuiWidgets));
      scene3D.addRenderableProvider(this::getRenderables);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);

      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
      scene3D.addRenderableProvider(planarRegionsGraphic::getRenderables);

      probeSphere = new RDXModelInstance(RDXModelBuilder.createSphere(0.005f, Color.VIOLET));
      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::probeTooltip);

      directory = new ImGuiDirectory(FootstepPlannerLogger.defaultLogsDirectory,
                                     logName -> footstepPlannerLog != null && footstepPlannerLog.getLogName().equals(logName),
                                     pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.DIRECTORY
                                        && pathEntry.path().getFileName().toString().endsWith(FootstepPlannerLogger.FOOTSTEP_PLANNER_LOG_POSTFIX),
                                     this::onLogSelected);

      goalGraphic = new RDXSphereAndArrowGraphic();
      goalGraphic.create(0.027, 0.027 * 6.0, Color.GREEN);
      footstepPlanGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      SideDependentList<Color> goalColors = new SideDependentList<>(new Color(0xc456bdff), new Color(0x56c4c4ff));
      SideDependentList<Color> startColors = new SideDependentList<>(new Color(0x566cc4ff), new Color(0xb6c456ff));
      for (RobotSide side : RobotSide.values)
      {
         RDXFootstepGraphic goalPoseGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints().get(side),
                                                                     goalColors.get(side));
         goalPoseGraphic.create();
         goalPoseGraphic.setupTooltip(panel3D, side.name() + " goal footstep");
         panel3D.addImGui3DViewPickCalculator(goalPoseGraphic::calculate3DViewPick);
         panel3D.addImGui3DViewInputProcessor(goalPoseGraphic::process3DViewInput);
         goalFootPoses.put(side, goalPoseGraphic);
         RDXFootstepGraphic startPoseGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints().get(side),
                                                                      startColors.get(side));
         startPoseGraphic.create();
         startPoseGraphic.setupTooltip(panel3D, side.name() + " start footstep");
         panel3D.addImGui3DViewPickCalculator(startPoseGraphic::calculate3DViewPick);
         panel3D.addImGui3DViewInputProcessor(startPoseGraphic::process3DViewInput);
         startFootPoses.put(side, startPoseGraphic);
      }
   }

   public void update()
   {
      if (logLoadedNotification.poll())
      {
         footstepPlannerLog = logLoadedNotification.read();
         footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(footstepPlannerLog.getStatusPacket().getFootstepDataList());
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlan, "Footstep plan"));

         rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlannerLog);
         rejectionReasonReport.update();

         // Move the camera to where the data is, so we don't have to find it.
         panel3D.getCamera3D().setCameraFocusPoint(footstepPlannerLog.getRequestPacket().getStartLeftFootPose().getPosition());
      }
      planarRegionsGraphic.update();
      footstepPlanGraphic.update();

      panel3D.setModelSceneMouseCollisionEnabled(probeMode.get());
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (probeMode.get())
      {
         tooltip.setInput(input);
         probeSphere.setPositionInWorldFrame(input.getPickPointInWorld());
         probePose.setToZero(ReferenceFrame.getWorldFrame());
         probePose.getPosition().set(input.getPickPointInWorld());
      }
   }

   public void renderImGuiWidgets()
   {
      directory.renderImGuiWidgets();

      ImGui.checkbox(labels.get("Probe mode"), probeMode);

      if (footstepPlannerLog != null)
      {
         ImGui.text("Loaded log:");
         ImGui.text(footstepPlannerLog.getLogName());

         goalPose.set(footstepPlannerLog.getStatusPacket().getGoalPose().getPosition(), footstepPlannerLog.getStatusPacket().getGoalPose().getOrientation());
         goalGraphic.setToPose(goalPose);
         goalFootPoses.get(RobotSide.LEFT).setPose(footstepPlannerLog.getRequestPacket().getGoalLeftFootPose());
         goalFootPoses.get(RobotSide.RIGHT).setPose(footstepPlannerLog.getRequestPacket().getGoalRightFootPose());
         startFootPoses.get(RobotSide.LEFT).setPose(footstepPlannerLog.getRequestPacket().getStartLeftFootPose());
         startFootPoses.get(RobotSide.RIGHT).setPose(footstepPlannerLog.getRequestPacket().getStartRightFootPose());

         ImGui.text("Number of planned footsteps: " + footstepPlan.getNumberOfSteps());
         ImGui.text("Requested initial stance side: " + RobotSide.fromByte(footstepPlannerLog.getRequestPacket().getRequestedInitialStanceSide()).name());
         ImGui.text("Goal distance proximity: " + footstepPlannerLog.getRequestPacket().getGoalDistanceProximity());
         ImGui.text("Goal yaw proximity: " + footstepPlannerLog.getRequestPacket().getGoalYawProximity());
         ImGui.text("Timeout: " + footstepPlannerLog.getRequestPacket().getTimeout());
         ImGui.text("Horizon length: " + footstepPlannerLog.getRequestPacket().getHorizonLength());
         ImGui.text("Assume flat ground: " + footstepPlannerLog.getRequestPacket().getAssumeFlatGround());
         ImGui.text("Snap goal steps: " + footstepPlannerLog.getRequestPacket().getSnapGoalSteps());
         ImGui.text("Planner request ID: " + footstepPlannerLog.getRequestPacket().getPlannerRequestId());
         ImGui.text("Perform A* search: " + footstepPlannerLog.getRequestPacket().getPerformAStarSearch());
         ImGui.text("Plan body path: " + footstepPlannerLog.getRequestPacket().getPlanBodyPath());
         ImGui.text("Result: " + FootstepPlanningResult.fromByte(footstepPlannerLog.getStatusPacket().getFootstepPlanningResult()).name());

         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            ImGui.text(String.format("Rejection %.3f%%: %s", rejectionPercentage, reason));
         }
      }
      else
      {
         ImGui.text("No log loaded.");
      }
   }

   private void onLogSelected(String logName)
   {
      footstepPlannerLog = null;
      ThreadTools.startAThread(() ->
      {
         FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
         FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load(new File(directory.getDirectoryName() + "/" + logName));
         if (loadResult == FootstepPlannerLogLoader.LoadResult.LOADED)
         {
            logLoadedNotification.set(logLoader.getLog());
         }
         else if (loadResult == FootstepPlannerLogLoader.LoadResult.ERROR)
         {
            LogTools.error("Error loading log");
         }

      }, "FootstepPlanLogLoading");
   }

   private void probeTooltip()
   {
      if (probeMode.get())
      {
         if (footstepPlannerLog != null)
         {
            probedFootFrames.get(RobotSide.LEFT).update(transformToWorld -> transformToWorld.set(footstepPlannerLog.getRequestPacket().getStartLeftFootPose()));
            probePose.changeFrame(probedFootFrames.get(RobotSide.LEFT).getReferenceFrame());
            String tooltipString = "Left start to here: " + EuclidCoreIOTools.getTuple3DString(probePose.getPosition());

            probedFootFrames.get(RobotSide.RIGHT).update(transformToWorld -> transformToWorld.set(footstepPlannerLog.getRequestPacket().getStartRightFootPose()));
            probePose.changeFrame(probedFootFrames.get(RobotSide.RIGHT).getReferenceFrame());
            tooltipString += "\nRight start to here: " + EuclidCoreIOTools.getTuple3DString(probePose.getPosition());

            tooltip.render(tooltipString, 2);
         }
         else
         {
            tooltip.render("No log loaded.", 2);
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.MODEL))
      {
         goalGraphic.getRenderables(renderables, pool);
         footstepPlanGraphic.getRenderables(renderables, pool);
         for (RDXFootstepGraphic goalFootPose : goalFootPoses)
         {
            goalFootPose.getRenderables(renderables, pool);
         }
         for (RDXFootstepGraphic startFootPose : startFootPoses)
         {
            startFootPose.getRenderables(renderables, pool);
         }
      }
      if (probeMode.get() && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         probeSphere.getRenderables(renderables, pool);
      }
   }
}
