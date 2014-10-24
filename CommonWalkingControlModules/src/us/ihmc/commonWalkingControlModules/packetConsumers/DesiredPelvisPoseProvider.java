package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements ObjectConsumer<PelvisPosePacket>, PelvisPoseProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private AtomicReference<FramePoint> desiredPelvisPosition = new AtomicReference<FramePoint>(new FramePoint(ReferenceFrame.getWorldFrame()));
   private AtomicReference<FrameOrientation> desiredPelvisOrientation = new AtomicReference<FrameOrientation>(new FrameOrientation(ReferenceFrame.getWorldFrame()));
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
   public FramePoint getDesiredPelvisPosition()
   {
      return desiredPelvisPosition.getAndSet(null);
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation()
   {
      return desiredPelvisOrientation.getAndSet(null);
   }

   // TODO That'd be nice to include the trajectory time in the packet
   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public void consumeObject(PelvisPosePacket object)
   {
      if (object == null)
         return;

      if (object.getPoint() != null)
         desiredPelvisPosition.set(new FramePoint(worldFrame, object.getPoint()));
      if (object.getQuaternion() != null)
         desiredPelvisOrientation.set(new FrameOrientation(worldFrame, object.getQuaternion()));

      trajectoryTime = object.getTrajectoryTime();
   }
}
