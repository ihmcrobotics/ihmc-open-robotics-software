package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameBasedMessage
{
   long getTrajectoryReferenceFrameId();
   long getDataReferenceFrameId();
   void setTrajectoryReferenceFrameId(long trajedtoryReferenceFrameId);
   void setTrajectoryReferenceFrameId(ReferenceFrame trajectoryReferenceFrame);
   void setDataReferenceFrameId(long expressedInReferenceFrameId);
   void setDataReferenceFrameId(ReferenceFrame expressedInReferenceFrame);

   default void checkIfTrajectoryFrameIdsMatch(long frameId, ReferenceFrame referenceFrame)
   {
      if(frameId != referenceFrame.getNameBasedHashCode())
      {
         String msg = "Argument's hashcode " + referenceFrame + " " +  referenceFrame.getNameBasedHashCode() + " does not match " + frameId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   default void checkIfFrameIdsMatch(long frameId, long otherReferenceFrameId)
   {
      if(frameId != otherReferenceFrameId)
      {
         String msg = "Argument's hashcode " + otherReferenceFrameId + " does not match " + frameId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   abstract public Vector3D getControlFramePosition();

   abstract public Quaternion getControlFrameOrientation();

   default public void getTransformFromBodyToControlFrame(RigidBodyTransform transformToPack)
   {
      transformToPack.set(getControlFrameOrientation(), getControlFramePosition());
   }

   default public boolean useCustomControlFrame()
   {
      return false;
   }
}
