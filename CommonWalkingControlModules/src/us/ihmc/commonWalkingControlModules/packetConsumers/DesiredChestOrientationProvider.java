package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredChestOrientationProvider implements ObjectConsumer<ChestOrientationPacket>, ChestOrientationProvider
{

   private final AtomicReference<FrameOrientation> desiredChestOrientation = new AtomicReference<FrameOrientation>(new FrameOrientation(ReferenceFrame.getWorldFrame()));
   private final ReferenceFrame chestOrientationFrame;

   public DesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame)
   {
      this.chestOrientationFrame = chestOrientationFrame;
   }

   @Override
   public boolean isNewChestOrientationInformationAvailable()
   {
      return desiredChestOrientation.get() != null;
   }

   @Override
   public FrameOrientation getDesiredChestOrientation()
   {
      return desiredChestOrientation.getAndSet(null);
   }

   public void consumeObject(ChestOrientationPacket object)
   {
      desiredChestOrientation.set(new FrameOrientation(ReferenceFrame.getWorldFrame(), object.getQuaternion()));
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }
}
