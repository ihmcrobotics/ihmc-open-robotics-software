package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.PersistentDetection;

/**
 * Also known as the "Tracker"
 */
public class BehaviorTreeSceneObjectExecutor extends BehaviorTreeSceneObjectState
{
   private PersistentDetection persistentDetection;

   public BehaviorTreeSceneObjectExecutor(long id, CRDTInfo crdtInfo, String type)
   {
      super(id, crdtInfo, type);
      //new PersistentDetection()
   }

   public void update()
   {

   }

   public void destroy()
   {

   }
}
