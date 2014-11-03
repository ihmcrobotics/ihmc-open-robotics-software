package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredChestOrientationProvider implements ObjectConsumer<ChestOrientationPacket>, ChestOrientationProvider
{

   private final AtomicReference<Quat4d> desiredOrientation = new AtomicReference<>();
   private final ReferenceFrame chestOrientationFrame;

   public DesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame)
   {
      this.chestOrientationFrame = chestOrientationFrame;
   }

   @Override
   public boolean isNewChestOrientationInformationAvailable()
   {
      return desiredOrientation.get() != null;
   }

   @Override
   public FrameOrientation getDesiredChestOrientation()
   {
      Quat4d orientation = desiredOrientation.getAndSet(null);
      if(orientation == null)
      {
         return null;
      }
      
      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation);
      frameOrientation.changeFrame(chestOrientationFrame);
      return frameOrientation;
   }

   public void consumeObject(ChestOrientationPacket object)
   {
      desiredOrientation.set(object.getQuaternion());
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }
}
