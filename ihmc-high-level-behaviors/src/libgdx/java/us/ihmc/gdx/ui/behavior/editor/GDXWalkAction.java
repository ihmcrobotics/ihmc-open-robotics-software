package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXPathControlRingGizmo;
import us.ihmc.gdx.ui.graphics.GDXFootstepGraphic;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

import java.util.List;
import java.util.UUID;

public class GDXWalkAction implements GDXBehaviorAction
{
   private GDXFootstepPlanGraphic footstepPlanGraphic;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2ControllerHelper;
   private List<ReferenceFrame> referenceFrameLibrary;
   private final SideDependentList<GDXFootstepGraphic> goalFeet = new SideDependentList<>();
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>();
   private FootstepPlanningModule footstepPlanner;
   private final GDXPathControlRingGizmo footstepPlannerGoalGizmo = new GDXPathControlRingGizmo();
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private final ImInt referenceFrameIndex = new ImInt();
   private String[] referenceFrameNames;
   private FootstepDataListMessage footstepDataListMessage;

   public void create(GDXFocusBasedCamera camera3D,
                      DRCRobotModel robotModel,
                      FootstepPlanningModule footstepPlanner,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper ros2ControllerHelper,
                      List<ReferenceFrame> referenceFrameLibrary)
   {
      this.footstepPlanner = footstepPlanner;
      footstepPlanGraphic = new GDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.referenceFrameLibrary = referenceFrameLibrary;
      referenceFrameNames = new String[referenceFrameLibrary.size()];
      for (int i = 0; i < referenceFrameLibrary.size(); i++)
      {
         referenceFrameNames[i] = referenceFrameLibrary.get(i).getName();
      }

      footstepPlannerGoalGizmo.create(camera3D);
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      for (RobotSide side : RobotSide.values)
      {
         GDXFootstepGraphic goalFootGraphic = new GDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(),
                                                                     side);
         goalFootGraphic.create();
         goalFeet.put(side, goalFootGraphic);
         goalFeetPoses.put(side, new FramePose3D());
      }
   }

   @Override
   public void update()
   {
      footstepPlannerGoalGizmo.updateTransforms();
      for (RobotSide side : RobotSide.values)
      {
         FramePose3D goalFootPose = goalFeetPoses.get(side);
         goalFootPose.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
         goalFootPose.getPosition().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
         goalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalFeet.get(side).setPose(goalFootPose);
      }
      footstepPlanGraphic.update();
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         footstepPlannerGoalGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.combo(labels.get("Reference frame"), referenceFrameIndex, referenceFrameNames))
      {
         FramePose3D poseToKeep = new FramePose3D();
         poseToKeep.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
         footstepPlannerGoalGizmo.setParentFrame(referenceFrameLibrary.get(referenceFrameIndex.get()));
         poseToKeep.changeFrame(footstepPlannerGoalGizmo.getGizmoFrame().getParent());
         poseToKeep.get(footstepPlannerGoalGizmo.getTransformToParent());
      }
      if (ImGui.button(labels.get("Plan")))
      {
         plan();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      if (selected.get())
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
      for (RobotSide side : RobotSide.values)
         goalFeet.get(side).getRenderables(renderables, pool);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("parentFrame", footstepPlannerGoalGizmo.getGizmoFrame().getParent().getName());
      JSONTools.toJSON(jsonNode, footstepPlannerGoalGizmo.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      String referenceFrameName = jsonNode.get("parentFrame").asText();
      for (int i = 0; i < referenceFrameLibrary.size(); i++)
      {
         ReferenceFrame referenceFrame = referenceFrameLibrary.get(i);
         if (referenceFrameName.equals(referenceFrame.getName()))
         {
            referenceFrameIndex.set(i);
            footstepPlannerGoalGizmo.setParentFrame(referenceFrame);
         }
      }

      JSONTools.toEuclid(jsonNode, footstepPlannerGoalGizmo.getTransformToParent());
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
   }

   public void plan()
   {
      double proximityToGoalToMaintainOrientation = 1.5;

      FramePose3D approachPointA = new FramePose3D();
      approachPointA.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
      approachPointA.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3DReadOnly midFeetUnderPelvisFramePose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
      double midFeetUnderPelvisYaw = midFeetUnderPelvisFramePose.getYaw();

      FrameVector3D walkingDirection = new FrameVector3D(ReferenceFrame.getWorldFrame());
      walkingDirection.set(approachPointA.getPosition());
      walkingDirection.sub(midFeetUnderPelvisFramePose.getPosition());
      walkingDirection.normalize();
      double pathToGoalYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());

      double distanceToGoal = approachPointA.getPosition().distanceXY(midFeetUnderPelvisFramePose.getPosition());
      double desiredHeading = AngleTools.computeAngleDifferenceMinusPiToPi(midFeetUnderPelvisYaw, pathToGoalYaw);
      approachPointA.getOrientation().setToYawOrientation(midFeetUnderPelvisYaw);

      RobotSide stanceSide = RobotSide.LEFT;
      FramePose3D leftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(leftFootPose, rightFootPose);
      // TODO: Set start footholds!!
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), approachPointA);
      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      LogTools.info("Stance side: {}", stanceSide.name());
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
         }
         LogTools.info("Footstep planning failure...");
         footstepDataListMessage = null;
      }
      else
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(),
                                                                                                 "Walk Action Planned"));

         double swingDuration = 1.2;
         double transferDuration = 0.8;
         footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlannerOutput.getFootstepPlan(),
                                                                       swingDuration,
                                                                       transferDuration);
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
   }

   @Override
   public void performAction()
   {
      plan();
      if (footstepDataListMessage != null)
      {
         ros2ControllerHelper.publishToController(footstepDataListMessage);
      }
   }

   @Override
   public ImBoolean getSelected()
   {
      return selected;
   }

   @Override
   public String getNameForDisplay()
   {
      return "Walk Goal";
   }
}
