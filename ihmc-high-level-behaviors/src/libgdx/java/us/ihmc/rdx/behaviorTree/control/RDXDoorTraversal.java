package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionState;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.actions.RDXActionNode;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.EuclidCoreMissingTools;

import javax.annotation.Nullable;
import java.util.List;

public class RDXDoorTraversal extends RDXBehaviorTreeNode<DoorTraversalState, DoorTraversalDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSliderDoubleWrapper lostGraspDetectionHandOpenAngleSlider;
   private final ImGuiSliderDoubleWrapper openedDoorHandleDistanceFromStartSlider;

   public RDXDoorTraversal(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new DoorTraversalState(id, rootNode.getState()), rootNode);

      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      lostGraspDetectionHandOpenAngleSlider = new ImGuiSliderDoubleWrapper("Lost grasp detection hand open angle", "",
                                                                           0.0, Math.toRadians(SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES),
                                                                           definition.getLostGraspDetectionHandOpenAngle()::getValue,
                                                                           definition.getLostGraspDetectionHandOpenAngle()::setValue);
      lostGraspDetectionHandOpenAngleSlider.addWidgetAligner(widgetAligner);

      openedDoorHandleDistanceFromStartSlider = new ImGuiSliderDoubleWrapper("Door handle distance from start", "%.2f",
                                                                           0.0, 1.50,
                                                                           definition.getOpenedDoorHandleDistanceFromStart()::getValue,
                                                                           definition.getOpenedDoorHandleDistanceFromStart()::setValue);
      openedDoorHandleDistanceFromStartSlider.addWidgetAligner(widgetAligner);
   }

   @Override
   public void update()
   {
      super.update();

      updateSubtree(this);
   }

   public void updateSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {

         }
         else
         {
            updateSubtree(child);
         }
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(definition.getClass().getSimpleName(), state.getID()));

      renderNodePresenceStatus(DoorTraversalState.SET_STATIC_FOR_APPROACH, state.getSetStaticForApproachActions());
      renderNodePresenceStatus(DoorTraversalState.SET_STATIC_FOR_GRASP, state.getSetStaticForGraspActions());
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

      lostGraspDetectionHandOpenAngleSlider.setWidgetText("%.1f%s".formatted(Math.toDegrees(definition.getLostGraspDetectionHandOpenAngle().getValue()),
                                                                             EuclidCoreMissingTools.DEGREE_SYMBOL));
      lostGraspDetectionHandOpenAngleSlider.renderImGuiWidget();

      openedDoorHandleDistanceFromStartSlider.setWidgetText("%.2f".formatted(definition.getOpenedDoorHandleDistanceFromStart().getValue()));
      openedDoorHandleDistanceFromStartSlider.renderImGuiWidget();

         boolean pullScrewPrimitiveIsExecuting = false;
         if (state.arePullRetryNodesPresent())
         {
            pullScrewPrimitiveIsExecuting = state.getPullScrewPrimitiveAction().getIsExecuting();
         }
         ImGui.beginDisabled(state.arePullRetryNodesPresent());
         ImGui.text("Pull screw primitive node: Executing: %b".formatted(pullScrewPrimitiveIsExecuting));
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

   private void renderNodePresenceStatus(String expectedName, List<WaitDurationActionState> nodes)
   {
      ImGui.text("%s: ".formatted(expectedName));
      ImGui.sameLine();
      if (nodes.isEmpty())
         ImGui.textColored(ImGuiTools.DARK_RED, "MISSING");
      else
         ImGui.textColored(ImGuiTools.DARK_GREEN, "FOUND %d".formatted(nodes.size()));
   }
}