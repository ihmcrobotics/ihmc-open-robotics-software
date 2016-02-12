package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisPosePacket;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements PelvisPoseProvider, PacketConsumer<PelvisPosePacket>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final AtomicBoolean goToHomePosition = new AtomicBoolean(false);
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final AtomicReference<FramePoint> desiredPelvisPosition = new AtomicReference<FramePoint>(null);
   private final AtomicReference<FrameOrientation> desiredPelvisOrientation = new AtomicReference<FrameOrientation>(null);
   private double trajectoryTime = Double.NaN;

   public DesiredPelvisPoseProvider()
   {
   }

   @Override
   public boolean checkForNewPosition()
   {
      return desiredPelvisPosition.get() != null;
   }

   @Override
   public boolean checkForNewOrientation()
   {
      return desiredPelvisOrientation.get() != null;
   }

   @Override
   public void clearOrientation()
   {
      desiredPelvisOrientation.set(null);
      goToHomeOrientation.set(false);
   }

   @Override
   public void clearPosition()
   {
      desiredPelvisPosition.set(null);
      goToHomePosition.set(false);
   }

   @Override
   public boolean checkForHomePosition()
   {
      return goToHomePosition.getAndSet(false);
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      return goToHomeOrientation.getAndSet(false);
   }

   @Override
   public FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame)
   {
      return desiredPelvisPosition.getAndSet(null);
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame)
   {
      FrameOrientation ret = desiredPelvisOrientation.getAndSet(null);

      if (ret == null)
         return null;

      ret.changeFrame(desiredPelvisFrame);
      return ret;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public void receivedPacket(PelvisPosePacket object)
   {
      if (object == null)
         return;

      trajectoryTime = object.getTrajectoryTime();

      // If go to home position requested, ignore the other commands.
      if (object.isToHomePosition())
      {
         goToHomePosition.set(true);
         goToHomeOrientation.set(true);
         return;
      }

      if (object.getPosition() != null)
         desiredPelvisPosition.set(new FramePoint(worldFrame, object.getPosition()));
      else
         desiredPelvisPosition.set(null);

      if (object.getOrientation() != null)
         desiredPelvisOrientation.set(new FrameOrientation(worldFrame, object.getOrientation()));
      else
         desiredPelvisOrientation.set(null);
   }
}
