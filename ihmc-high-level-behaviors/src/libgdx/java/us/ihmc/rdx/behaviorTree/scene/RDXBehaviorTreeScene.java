package us.ihmc.rdx.behaviorTree.scene;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXPanel panel = new RDXPanel("Scene", this::renderImGuiWidgets);

   public RDXBehaviorTreeScene(ROS2SyncedRobotModel syncedRobot, RDXPanel parentPanel)
   {
      super(syncedRobot);

      parentPanel.addChild(panel);
   }

   private void renderImGuiWidgets()
   {

   }
}
