package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TrackingWeightsCommand.BodyWeights;

public class TrackingWeightsMessage extends TrackablePacket<TrackingWeightsMessage>
{
   public BodyWeights bodyWeights;
   
   public TrackingWeightsMessage()
   {
      this.bodyWeights = BodyWeights.STANDARD;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }
   
   public TrackingWeightsMessage(BodyWeights bodyWeights)
   {
      this.bodyWeights = bodyWeights;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public void setTrackingWeightsMessage(BodyWeights bodyWeights)
   {
      this.bodyWeights = bodyWeights;
   }
   
   public BodyWeights getBodyWeights()
   {
      return this.bodyWeights;
   }

   @Override
   public boolean epsilonEquals(TrackingWeightsMessage other, double epsilon)
   {
      return true;
   }

}
