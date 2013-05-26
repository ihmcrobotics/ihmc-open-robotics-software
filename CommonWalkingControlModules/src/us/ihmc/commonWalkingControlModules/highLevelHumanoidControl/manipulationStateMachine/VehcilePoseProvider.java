package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class VehcilePoseProvider implements ObjectConsumer<VehiclePosePacket>
{
   private final Object synchronizationObject = new Object();
   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private boolean hasNewPose;

   public void consumeObject(VehiclePosePacket object)
   {
      synchronized (synchronizationObject)
      {
         framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
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

   public FramePoint getPosition()
   {
      return framePose.getPostionCopy();
   }

   public FrameOrientation getOrientation()
   {
      return framePose.getOrientationCopy();
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

