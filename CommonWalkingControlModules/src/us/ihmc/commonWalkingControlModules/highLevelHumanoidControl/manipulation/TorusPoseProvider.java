package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import us.ihmc.commonWalkingControlModules.packets.TorusPosePacket;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;

/**
 * @author twan
 *         Date: 5/14/13
 */
public class TorusPoseProvider implements ObjectConsumer<TorusPosePacket>
{
   private final Object synchronizationObject = new Object();
   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private double fingerHoleRadius;
   private boolean hasNewPose;

   public void consumeObject(TorusPosePacket object)
   {
      synchronized (synchronizationObject)
      {
         framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
         fingerHoleRadius = object.getFingerHoleRadius();
         hasNewPose = true;
      }
   }

   public boolean checkForNewPose()
   {
      synchronized (synchronizationObject)
      {
         return hasNewPose;
      }
   }

   public double getFingerHoleRadius()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return fingerHoleRadius;
      }
   }

   public FrameVector getNormal()
   {
      // TODO: Twan, please review this "getting" of the normal
      AxisAngle4d axisAngle4d = new AxisAngle4d();
      Quat4d quat4d = framePose.getOrientationCopy().getQuaternion();
      axisAngle4d.set(quat4d);

      FrameVector normal = new FrameVector(framePose.getReferenceFrame(), axisAngle4d.getX(), axisAngle4d.getY(), axisAngle4d.getZ());

      return normal;
   }

   public FramePoint getPosition()
   {
      return framePose.getPostionCopy();
   }

   public FramePose getFramePose()
   {
      // TODO: this is NOT a defensive copy. Should we make one?
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return framePose;
      }
   }
}
