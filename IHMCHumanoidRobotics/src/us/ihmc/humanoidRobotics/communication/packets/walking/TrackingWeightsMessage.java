package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TrackingWeightsCommand.BodyWeights;

public class TrackingWeightsMessage extends Packet<TrackingWeightsMessage>
{
   BodyWeights bodyWeights;
   
   public TrackingWeightsMessage(BodyWeights bodyWeights)
   {
      this.bodyWeights = bodyWeights;
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
