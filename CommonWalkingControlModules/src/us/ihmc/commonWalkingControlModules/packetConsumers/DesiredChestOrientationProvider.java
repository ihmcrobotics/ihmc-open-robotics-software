package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4d;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DesiredChestOrientationProvider implements PacketConsumer<ChestOrientationPacket>, ChestOrientationProvider
{
   private final AtomicReference<Quat4d> desiredOrientation = new AtomicReference<>();
   private final AtomicDouble trajectoryTime = new AtomicDouble();
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);

   private final HumanoidGlobalDataProducer globalDataProducer;

   public DesiredChestOrientationProvider(double defaultTrajectoryTime, HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
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
      if (orientation == null)
         return null;

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation);
      return frameOrientation;
   }

   @Override
   public void receivedPacket(ChestOrientationPacket object)
   {
      if (object == null)
         return;

      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateChestOrientationPacket(object);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(ChestOrientationPacket.class, errorMessage);
            return;
         }
      }

      double desiredTrajectoryTime = object.getTrajectoryTime();
      desiredTrajectoryTime = MathTools.clipToMinMax(desiredTrajectoryTime, 0.0001, 30.0);
      trajectoryTime.set(desiredTrajectoryTime);

      // If go to home orientation requested, ignore the other commands.
      if (object.isToHomeOrientation())
      {
         goToHomeOrientation.set(true);
         return;
      }

      desiredOrientation.set(object.getOrientation());
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime.get();
   }
}
