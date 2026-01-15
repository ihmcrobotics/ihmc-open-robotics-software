package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.RDXBaseUI;

/**
 * Extends RDXBehaviorTreeSceneObject with an additional persistent detection
 * specifically for door panel visualization, corresponding to BehaviorTreeSceneDoorPanelExecutor.
 */
public class RDXBehaviorTreeSceneDoorPanel extends RDXBehaviorTreeSceneObject
{
   private final PersistentDetectionStatusMessage doorPanelPersistentDetection = new PersistentDetectionStatusMessage();

   public RDXBehaviorTreeSceneDoorPanel(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      doorPanelPersistentDetection.set(message.getPersistentDetection());
   }

   public PersistentDetectionStatusMessage getDoorPanelPersistentDetection()
   {
      return doorPanelPersistentDetection;
   }
}
