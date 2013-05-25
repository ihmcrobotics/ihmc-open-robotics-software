package us.ihmc.commonWalkingControlModules.controlModules.desiredChestOrientation;

import us.ihmc.commonWalkingControlModules.controlModules.spine.ChestOrientationPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredChestOrientationProvider implements ObjectConsumer<ChestOrientationPacket>
{

   private FrameOrientation desiredChestOrientation;
   private Boolean hasNewPose;

   public DesiredChestOrientationProvider()
   {
      desiredChestOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
      hasNewPose = false;
   }

   public synchronized boolean checkForNewPose()
   {
      return hasNewPose;
   }

   public synchronized FrameOrientation getDesiredChestOrientation()
   {
      hasNewPose = false;
      
      return desiredChestOrientation;
   }

   public void consumeObject(ChestOrientationPacket object)
   {
      System.out.println("YOUHOUUUUUUUUUUUUUUUUUUUUUUUUUUU");
      hasNewPose = true;
      desiredChestOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), object.getQuaternion());
   }
}
