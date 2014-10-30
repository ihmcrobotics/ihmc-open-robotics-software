package us.ihmc.commonWalkingControlModules.packetProducers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;

public class HandPoseStatusProducer
{
   private final GlobalDataProducer objectCommunicator;

   public HandPoseStatusProducer(GlobalDataProducer objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }

   public void sendStatus(Point3d currentPosePosition, Quat4d currentPoseOrientationInWorldFrame, Point3d desiredPosePosition,
         Quat4d desiredPoseOrientationInWorldFrame)
   {
      HandPoseStatus handPoseStatus = new HandPoseStatus(currentPosePosition, currentPoseOrientationInWorldFrame, desiredPosePosition,
            desiredPoseOrientationInWorldFrame);
      objectCommunicator.queueDataToSend(handPoseStatus);
   }

}
