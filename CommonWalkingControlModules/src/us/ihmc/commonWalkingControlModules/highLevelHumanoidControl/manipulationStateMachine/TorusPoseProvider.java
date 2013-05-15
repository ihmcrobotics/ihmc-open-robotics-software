package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

/**
 * @author twan
 *         Date: 5/14/13
 */
public class TorusPoseProvider implements ObjectConsumer<TorusPosePacket>
{
   private final FrameVector normal = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FramePoint origin = new FramePoint(ReferenceFrame.getWorldFrame());
   private double radius;
   private boolean hasNewPose;

   public void consumeObject(TorusPosePacket object)
   {
      normal.set(object.getNormal());
      origin.set(object.getPosition());
      radius = object.getRadius();
      hasNewPose = true;
   }

   public synchronized boolean checkForNewPose()
   {
      return hasNewPose;
   }

   public synchronized double getRadius()
   {
      return radius;
   }

   public FrameVector getNormal()
   {
      return normal;
   }

   public FramePoint getOrigin()
   {
      return origin;
   }
}
