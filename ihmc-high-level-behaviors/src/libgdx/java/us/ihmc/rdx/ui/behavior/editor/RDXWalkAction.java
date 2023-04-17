package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPathControlRingGizmo;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

import java.util.UUID;

public class RDXWalkAction implements RDXBehaviorAction
{
   private RDXFootstepPlanGraphic footstepPlanGraphic;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2ControllerHelper;
   private ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>();
   private FootstepPlanningModule footstepPlanner;
   private final RDXPathControlRingGizmo footstepPlannerGoalGizmo = new RDXPathControlRingGizmo();
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private FootstepDataListMessage footstepDataListMessage;
   private final SideDependentList<ImBoolean> editGoalFootPoses = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> editGoalFootGizmos = new SideDependentList<>();
   private final ImDouble swingDuration = new ImDouble(1.2);
   private final ImDouble transferDuration = new ImDouble(0.8);

   public void create(RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      FootstepPlanningModule footstepPlanner,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper ros2ControllerHelper,
                      ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.footstepPlanner = footstepPlanner;
      footstepPlanGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);

      footstepPlannerGoalGizmo.create(panel3D.getCamera3D());
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      for (RobotSide side : RobotSide.values)
      {
         editGoalFootPoses.put(side, new ImBoolean(false));
         RDXPose3DGizmo footGizmo = new RDXPose3DGizmo(footstepPlannerGoalGizmo.getGizmoFrame());
         footGizmo.create(panel3D);
         editGoalFootGizmos.put(side, footGizmo);

         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(),
                                                                     side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);
         FramePose3D goalFootPose = new FramePose3D();
         goalFootPose.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
         goalFootPose.getPosition().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
         goalFootPose.get(footGizmo.getTransformToParent());
         goalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalFootGraphic.setPose(goalFootPose);
         goalFeetPoses.put(side, goalFootPose);
      }
   }

   @Override
   public void update()
   {
      if (!selected.get())
         editGoalFootPoses.forEach(imBoolean -> imBoolean.set(false));

      footstepPlannerGoalGizmo.updateTransforms();
      for (RobotSide side : RobotSide.values)
      {
         editGoalFootGizmos.get(side).update();

         FramePose3D goalFootPose = goalFeetPoses.get(side);
         goalFootPose.setToZero(editGoalFootGizmos.get(side).getGizmoFrame());
         goalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalFeetGraphics.get(side).setPose(goalFootPose);
      }
      footstepPlanGraphic.update();
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         boolean goalFootEditingEnabled = false;
         for (RobotSide side : RobotSide.values)
         {
            if (editGoalFootPoses.get(side).get())
            {
               goalFootEditingEnabled = true;
               editGoalFootGizmos.get(side).calculate3DViewPick(input);
            }
         }
         if (!goalFootEditingEnabled)
            footstepPlannerGoalGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         boolean goalFootEditingEnabled = false;
         for (RobotSide side : RobotSide.values)
         {
            if (editGoalFootPoses.get(side).get())
            {
               goalFootEditingEnabled = true;
               editGoalFootGizmos.get(side).process3DViewInput(input);
            }
         }
         if (!goalFootEditingEnabled)
            footstepPlannerGoalGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (referenceFrameLibraryCombo.render())
      {
         FramePose3D poseToKeep = new FramePose3D();
         poseToKeep.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
         updateParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseToKeep.changeFrame(footstepPlannerGoalGizmo.getGizmoFrame().getParent());
         poseToKeep.get(footstepPlannerGoalGizmo.getTransformToParent());
      }
      if (ImGui.button(labels.get("Plan")))
      {
         plan();
      }
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         ImGui.checkbox(labels.get("Edit " + side.getPascalCaseName()), editGoalFootPoses.get(side));
         if (side == RobotSide.LEFT)
            ImGui.sameLine();
      }
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Swing duration"), swingDuration);
      ImGui.inputDouble(labels.get("Transfer duration"), transferDuration);
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      if (selected.get())
      {
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
         for (RobotSide side : RobotSide.values)
         {
            if (editGoalFootPoses.get(side).get())
            {
               editGoalFootGizmos.get(side).getRenderables(renderables, pool);
            }
         }
      }
      for (RobotSide side : RobotSide.values)
         goalFeetGraphics.get(side).getRenderables(renderables, pool);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("parentFrame", footstepPlannerGoalGizmo.getGizmoFrame().getParent().getName());
      JSONTools.toJSON(jsonNode, footstepPlannerGoalGizmo.getTransformToParent());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, editGoalFootGizmos.get(side).getTransformToParent());
      }
      jsonNode.put("swingDuration", swingDuration.get());
      jsonNode.put("transferDuration", transferDuration.get());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      String referenceFrameName = jsonNode.get("parentFrame").asText();
      if (referenceFrameLibraryCombo.setSelectedReferenceFrame(referenceFrameName))
      {
         updateParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
      }
      JSONTools.toEuclid(jsonNode, footstepPlannerGoalGizmo.getTransformToParent());
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, editGoalFootGizmos.get(side).getTransformToParent());
      }
      if (jsonNode.has("swingDuration"))
         swingDuration.set(jsonNode.get("swingDuration").asDouble());
      if (jsonNode.has("transferDuration"))
         transferDuration.set(jsonNode.get("transferDuration").asDouble());
   }

   private void updateParentFrame(ReferenceFrame newParentFrame)
   {
      footstepPlannerGoalGizmo.setParentFrame(newParentFrame);
      for (RobotSide side : RobotSide.values)
      {
         editGoalFootGizmos.get(side).setParentFrame(footstepPlannerGoalGizmo.getGizmoFrame());
      }
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
   }

   public void plan()
   {
      FramePose3D leftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlannerParameters.setFinalTurnProximity(1.0);

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setStartFootPoses(leftFootPose, rightFootPose);
      // TODO: Set start footholds!!
      for (RobotSide side : RobotSide.values)
      {
         footstepPlannerRequest.setGoalFootPose(side, goalFeetPoses.get(side));
      }

      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      double idealFootstepLength = 0.5;
      footstepPlanner.getFootstepPlannerParameters().setIdealFootstepLength(idealFootstepLength);
      footstepPlanner.getFootstepPlannerParameters().setMaximumStepReach(idealFootstepLength);
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(), "FootstepPlanLogDeletion");

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

         footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlannerOutput.getFootstepPlan(),
                                                                       swingDuration.get(),
                                                                       transferDuration.get());
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
