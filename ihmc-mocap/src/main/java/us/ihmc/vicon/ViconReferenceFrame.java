package us.ihmc.vicon;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class ViconReferenceFrame extends ReferenceFrame
{
   private final ViconClient viconClient;
   private final String bodyName;
   private boolean dataValid = false;
   private long lastUpdateTimeStamp = 0;

   private final Quaternion bodyToWorldQuaternion = new Quaternion();
   private final Vector3D bodyToWorldTranslation = new Vector3D();
   private final RigidBodyTransform bodyToWorldTransform = new RigidBodyTransform();

   public ViconReferenceFrame(String bodyName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, ViconClient viconClient)
   {
      super(bodyName.replace(":", "_"), parentFrame, transformToParent, false, false);
      this.bodyName = bodyName;
      this.viconClient = viconClient;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      QuaternionPose pose = viconClient.getQuaternionPose(bodyName);
      if (pose == null)
      {
         pose = new QuaternionPose();
      }

      setDataValid(pose.dataValid);
      bodyToWorldQuaternion.set(pose.qx, pose.qy, pose.qz, pose.qw);
      bodyToWorldTransform.setRotationAndZeroTranslation(bodyToWorldQuaternion);
      bodyToWorldTranslation.set(pose.xPosition, pose.yPosition, pose.zPosition);
      bodyToWorldTransform.setTranslation(bodyToWorldTranslation);
      transformToParent.set(bodyToWorldTransform);
      setLastUpdateTimeStamp(viconClient.getModelReadingTimeStamp(bodyName));
   }

   protected synchronized void setDataValid(boolean dataValid)
   {
      this.dataValid = dataValid;
   }

   public synchronized boolean isDataValid()
   {
      return dataValid;
   }

   public long getLastUpdateTimeStamp()
   {
      return lastUpdateTimeStamp;
   }

   public void setLastUpdateTimeStamp(long lastUpdateTimeStamp)
   {
      this.lastUpdateTimeStamp = lastUpdateTimeStamp;
   }

}
