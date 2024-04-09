package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
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

public class RDXDoorTraversal extends RDXBehaviorTreeNode<DoorTraversalState, DoorTraversalDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DoorTraversalState state;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiSliderDoubleWrapper handOpenAngleFailureSlider;

   public RDXDoorTraversal(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.syncedRobot = syncedRobot;

      getDefinition().setName("Door traversal");
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      handOpenAngleFailureSlider = new ImGuiSliderDoubleWrapper("Failure Hand Open Angle", "", 0.0, Math.toRadians(SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES),
                                                         getDefinition()::getMinimumHandOpenAngle,
                                                         getDefinition()::setMinimumHandOpenAngle);
      handOpenAngleFailureSlider.addWidgetAligner(widgetAligner);
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

      if (state.isTreeStructureValid())
      {
         handOpenAngleFailureSlider.setWidgetText("%.1f%s".formatted(Math.toDegrees(getDefinition().getMinimumHandOpenAngle()), EuclidCoreMissingTools.DEGREE_SYMBOL));
         handOpenAngleFailureSlider.renderImGuiWidget();
         ImGui.text("Pull screw primitive node: Executing: %b".formatted(state.getPullScrewPrimitiveAction().getIsExecuting()));
      }
      else
      {
         ImGui.textColored(ImGuiTools.DARK_RED, "Expected node(s) could not be found by name.");
      }

      super.renderNodeSettingsWidgets();
   }
}