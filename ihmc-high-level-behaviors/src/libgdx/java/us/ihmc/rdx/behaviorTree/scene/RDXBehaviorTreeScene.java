package us.ihmc.rdx.behaviorTree.scene;

import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXPanel panel = new RDXPanel("Scene", this::renderImGuiWidgets);

   public RDXBehaviorTreeScene(RDXPanel parentPanel)
   {
      parentPanel.addChild(panel);
   }

   private void renderImGuiWidgets()
   {

   }
}
