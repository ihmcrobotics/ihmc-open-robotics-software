package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * @author twan
 *         Date: 5/14/13
 */
public class TorusPoseProvider implements ObjectConsumer<TorusPosePacket>
{
   private final Object synchronizationObject = new Object();
   private final FrameVector normal = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FramePoint origin = new FramePoint(ReferenceFrame.getWorldFrame());
   private double radius;
   private boolean hasNewPose;
   private double desiredRotationAngle;

   public void consumeObject(TorusPosePacket object)
   {
      synchronized (synchronizationObject)
      {
         normal.set(object.getNormal());
         origin.set(object.getPosition());
         radius = object.getRadius();
         desiredRotationAngle = object.getDesiredRotationAngle();
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

   public double getRadius()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return radius;
      }
   }

   public FrameVector getNormal()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return normal;
      }
   }

   public FramePoint getOrigin()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return origin;
      }
   }

   public double getDesiredRotationAngle()
   {
      synchronized (synchronizationObject)
      {
         return desiredRotationAngle;
      }
   }
}
