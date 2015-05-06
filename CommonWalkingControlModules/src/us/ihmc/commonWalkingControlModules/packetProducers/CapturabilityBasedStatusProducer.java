package us.ihmc.commonWalkingControlModules.packetProducers;

import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class CapturabilityBasedStatusProducer
{
   private final GlobalDataProducer objectCommunicator;

   public CapturabilityBasedStatusProducer(GlobalDataProducer objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }

   public void sendStatus(FramePoint2d capturePoint2d, FramePoint2d desiredCapturePoint2d, SideDependentList<FrameConvexPolygon2d> footSupportPolygons)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus(capturePoint2d, desiredCapturePoint2d, footSupportPolygons.get(RobotSide.LEFT), footSupportPolygons.get(RobotSide.RIGHT));
      objectCommunicator.queueDataToSend(capturabilityBasedStatus);
   }
}
