package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredChestOrientationProvider implements ObjectConsumer<ChestOrientationPacket>
{

   private final AtomicReference<FrameOrientation> desiredChestOrientation = new AtomicReference<FrameOrientation>(new FrameOrientation(ReferenceFrame.getWorldFrame()));

   public DesiredChestOrientationProvider()
   {
   }

   public boolean checkForNewPose()
   {
      return desiredChestOrientation.get() != null;
   }

   public FrameOrientation getDesiredChestOrientation()
   {
      return desiredChestOrientation.getAndSet(null);
   }

   public void consumeObject(ChestOrientationPacket object)
   {
      desiredChestOrientation.set(new FrameOrientation(ReferenceFrame.getWorldFrame(), object.getQuaternion()));
   }
}
