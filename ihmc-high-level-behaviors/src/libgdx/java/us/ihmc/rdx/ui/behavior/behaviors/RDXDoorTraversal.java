package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.door.DoorTraversalState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class RDXDoorTraversal extends RDXBehaviorTreeNode<DoorTraversalState, DoorTraversalDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DoorTraversalState state;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiSliderDoubleWrapper lostGraspDetectionHandOpenAngleSlider;
   private final ImGuiSliderDoubleWrapper openedDoorHandleDistanceFromStartSlider;

   public RDXDoorTraversal(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.syncedRobot = syncedRobot;

      getDefinition().setName("Door traversal");
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      lostGraspDetectionHandOpenAngleSlider = new ImGuiSliderDoubleWrapper("Lost grasp detection hand open angle", "",
                                                                           0.0, Math.toRadians(SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES),
                                                                           getDefinition().getLostGraspDetectionHandOpenAngle()::getValue,
                                                                           getDefinition().getLostGraspDetectionHandOpenAngle()::setValue);
      lostGraspDetectionHandOpenAngleSlider.addWidgetAligner(widgetAligner);

      openedDoorHandleDistanceFromStartSlider = new ImGuiSliderDoubleWrapper("Door handle distance from start", "%.2f",
                                                                           0.0, 1.50,
                                                                           getDefinition().getOpenedDoorHandleDistanceFromStart()::getValue,
                                                                           getDefinition().getOpenedDoorHandleDistanceFromStart()::setValue);
      openedDoorHandleDistanceFromStartSlider.addWidgetAligner(widgetAligner);
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);
   }

   public void updateActionSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(getDefinition().getClass().getSimpleName(), getState().getID()));

      renderNodePresenceStatus(DoorTraversalState.STABILIZE_DETECTION, state.getStabilizeDetectionAction());
      renderNodePresenceStatus(DoorTraversalState.WAIT_TO_OPEN_RIGHT_HAND, state.getWaitToOpenRightHandAction());
      renderNodePresenceStatus(DoorTraversalState.POST_GRASP_HANDLE, state.getPostGraspEvaluationAction());
      renderNodePresenceStatus(DoorTraversalState.POST_PULL_DOOR, state.getPostPullDoorEvaluationAction());
      renderNodePresenceStatus(DoorTraversalState.PULL_SCREW_PRIMITIVE, state.getPullScrewPrimitiveAction());

      ImGui.text("Pull door retry: ");
      ImGui.sameLine();
      if (state.arePullRetryNodesPresent())
         ImGui.textColored(ImGuiTools.DARK_GREEN, "ENABLED");
      else
         ImGui.textColored(ImGuiTools.DARK_RED, "DISABLED");

      lostGraspDetectionHandOpenAngleSlider.setWidgetText("%.1f%s".formatted(Math.toDegrees(getDefinition().getLostGraspDetectionHandOpenAngle().getValue()),
                                                                             EuclidCoreMissingTools.DEGREE_SYMBOL));
      lostGraspDetectionHandOpenAngleSlider.renderImGuiWidget();

      openedDoorHandleDistanceFromStartSlider.setWidgetText("%.2f".formatted(getDefinition().getOpenedDoorHandleDistanceFromStart().getValue()));
      openedDoorHandleDistanceFromStartSlider.renderImGuiWidget();

      boolean pullScrewPrimitiveIsExecuting = false;
      if (state.arePullRetryNodesPresent())
         pullScrewPrimitiveIsExecuting = state.getPullScrewPrimitiveAction().getIsExecuting();
      ImGui.beginDisabled(state.arePullRetryNodesPresent());
      ImGui.text("Pull screw primitive node: Executing: %b".formatted(state.getPullScrewPrimitiveAction().getIsExecuting()));
      ImGui.endDisabled();

      ImGui.text("Door hinge joint angle: %.2f%s".formatted(Math.toDegrees(state.getDoorHingeJointAngle().getValue()), EuclidCoreMissingTools.DEGREE_SYMBOL));

      super.renderNodeSettingsWidgets();
   }

   private void renderNodePresenceStatus(String expectedName, @Nullable BehaviorTreeNodeState<?> node)
   {
      ImGui.text("%s: ".formatted(expectedName));
      ImGui.sameLine();
      if (node == null)
         ImGui.textColored(ImGuiTools.DARK_RED, "MISSING");
      else
         ImGui.textColored(ImGuiTools.DARK_GREEN, "FOUND");
   }
}