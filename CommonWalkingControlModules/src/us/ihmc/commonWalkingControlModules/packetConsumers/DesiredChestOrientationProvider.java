package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.google.common.util.concurrent.AtomicDouble;

public class DesiredChestOrientationProvider implements PacketConsumer<ChestOrientationPacket>, ChestOrientationProvider
{
   private final AtomicReference<Quat4d> desiredOrientation = new AtomicReference<>();
   private final AtomicDouble trajectoryTime = new AtomicDouble();
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final ReferenceFrame chestOrientationFrame;

   public DesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame, double defaultTrajectoryTime)
   {
      this.chestOrientationFrame = chestOrientationFrame;
      trajectoryTime.set(defaultTrajectoryTime);
   }

   @Override
   public boolean checkForNewChestOrientation()
   {
      return desiredOrientation.get() != null;
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      return goToHomeOrientation.getAndSet(false);
   }

   @Override
   public FrameOrientation getDesiredChestOrientation()
   {
      Quat4d orientation = desiredOrientation.getAndSet(null);
      if (orientation == null) return null;

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation);
      frameOrientation.changeFrame(chestOrientationFrame);
      return frameOrientation;
   }

   public void receivedPacket(ChestOrientationPacket object)
   {
      if (object == null) return;

      trajectoryTime.set(object.getTrajectoryTime());

      // If go to home orientation requested, ignore the other commands.
      if (object.isToHomeOrientation())
      {
         goToHomeOrientation.set(true);
         return;
      }

      desiredOrientation.set(object.getOrientation());
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
