package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

/**
 * Also known as the "Tracker"
 */
public class BehaviorTreeSceneObjectExecutor extends BehaviorTreeSceneObjectState
{
   private PersistentDetection persistentDetection;

   private final FramePose3D detectionPose = new FramePose3D();

   public BehaviorTreeSceneObjectExecutor(long id, CRDTInfo crdtInfo, IsaacROSFoundationPoseObject objectType)
   {
      super(id, crdtInfo, objectType);
   }

   public void update(ReferenceFrame cameraFrame)
   {
      if (persistentDetection != null && persistentDetection.isStable())
      {
         detectionPose.setToZero(cameraFrame);
         detectionPose.set(persistentDetection.getFilteredTransformToCamera());
         detectionPose.changeFrame(ReferenceFrame.getWorldFrame());
         transform.setValue(detectionPose, 1e-5);
      }
   }

   public void destroy()
   {

   }

   public PersistentDetection getPersistentDetection()
   {
      return persistentDetection;
   }

   public void setPersistentDetection(PersistentDetection persistentDetection)
   {
      this.persistentDetection = persistentDetection;
   }
}
