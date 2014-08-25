package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packets.PelvisOrientationPacket;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;


/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements ObjectConsumer<PelvisOrientationPacket>
{

   private AtomicReference<FramePose> desiredPelvisPose = new AtomicReference<FramePose>(new FramePose(ReferenceFrame.getWorldFrame()));

   public DesiredPelvisPoseProvider()
   {
   }

   public boolean checkForNewPose()
   {
      return desiredPelvisPose.get() != null;
   }

   public FramePose getDesiredPelvisPose()
   {
      return desiredPelvisPose.getAndSet(null);
   }

   public void consumeObject(PelvisOrientationPacket object)
   {
      desiredPelvisPose.set(new FramePose(ReferenceFrame.getWorldFrame(), object.getPoint(), object.getQuaternion()));
   }
}
