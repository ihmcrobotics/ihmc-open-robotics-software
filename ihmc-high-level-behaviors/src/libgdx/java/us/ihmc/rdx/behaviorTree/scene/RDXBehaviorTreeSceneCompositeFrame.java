package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import imgui.ImGui;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXBehaviorTreeSceneCompositeFrame extends RDXBehaviorTreeSceneObject
{
   public RDXBehaviorTreeSceneCompositeFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);
   }

   @Override
   protected void renderDetectionInfo()
   {
      ImGui.text("Frame A: %s".formatted(getCompositeFrameA()));
      ImGui.text("Frame B: %s".formatted(getCompositeFrameB()));
      ImGui.text("Type: %s".formatted(getCompositeFrameType()));
      ImGui.text("Distance from B: %.2f".formatted(getCompositeDistance()));
   }
}
