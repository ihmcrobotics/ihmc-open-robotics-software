package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4d;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DesiredChestOrientationProvider implements ObjectConsumer<ChestOrientationPacket>, ChestOrientationProvider
{

   private final AtomicReference<Quat4d> desiredOrientation = new AtomicReference<>();
   private final AtomicDouble trajectoryTime = new AtomicDouble();
   private final ReferenceFrame chestOrientationFrame;

   public DesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame, double defaultTrajectoryTime)
   {
      this.chestOrientationFrame = chestOrientationFrame;
      trajectoryTime.set(defaultTrajectoryTime);
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
      trajectoryTime.set(object.getTrajectoryTime());
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime.get();
   }
}
