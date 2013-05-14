package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import us.ihmc.utilities.math.geometry.FramePose;
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
   private FramePose torusPose;
   private boolean hasNewPose;

   public void consumeObject(TorusPosePacket object)
   {
      Quat4d orientation = object.getOrientation();
      Point3d position = object.getPosition();

      torusPose = new FramePose(ReferenceFrame.getWorldFrame(), position, orientation);
      hasNewPose = true;
   }

   public synchronized boolean checkForNewPose()
   {
      return hasNewPose;
   }

   public synchronized FramePose getTorusPose()
   {
      hasNewPose = false;
      return torusPose;
   }
}
