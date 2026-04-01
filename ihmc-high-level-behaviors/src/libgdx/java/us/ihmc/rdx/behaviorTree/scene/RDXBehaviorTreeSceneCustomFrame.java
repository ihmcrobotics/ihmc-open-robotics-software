package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import imgui.ImGui;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXBehaviorTreeSceneCustomFrame extends RDXBehaviorTreeSceneObject
{
   public RDXBehaviorTreeSceneCustomFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);
   }

   @Override
   protected void renderDetectionInfo()
   {
      ImGui.text("Frame A: %s".formatted(getFrameA()));
      ImGui.text("Frame B: %s".formatted(getFrameB()));
      ImGui.text("Distance from B: %.2f".formatted(getDistance()));
   }
}
