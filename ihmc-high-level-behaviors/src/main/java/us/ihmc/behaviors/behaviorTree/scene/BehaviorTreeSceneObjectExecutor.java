package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

/**
 * Also known as the "Tracker"
 */
public class BehaviorTreeSceneObjectExecutor extends BehaviorTreeSceneObjectState
{
   private PersistentDetection persistentDetection;

   public BehaviorTreeSceneObjectExecutor(long id, CRDTInfo crdtInfo, IsaacROSFoundationPoseObject objectType)
   {
      super(id, crdtInfo, objectType);
   }

   public void update()
   {
      persistentDetection = null;
   }

   public void destroy()
   {

   }
}
