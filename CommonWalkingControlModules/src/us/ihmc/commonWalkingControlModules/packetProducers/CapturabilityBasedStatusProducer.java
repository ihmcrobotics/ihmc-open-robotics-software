package us.ihmc.commonWalkingControlModules.packetProducers;

import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.robotSide.RobotSide;

public class CapturabilityBasedStatusProducer
{
   private final GlobalDataProducer objectCommunicator;

   public CapturabilityBasedStatusProducer(GlobalDataProducer objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }

   public void sendStatus(FramePoint2d capturePoint2d, FramePoint2d desiredCapturePoint2d, FrameConvexPolygon2d supportPolygon, RobotSide supportLeg)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus(capturePoint2d, desiredCapturePoint2d, supportPolygon, supportLeg);
      objectCommunicator.queueDataToSend(capturabilityBasedStatus);
   }
}
