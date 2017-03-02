package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.TrackingWeightsMessage;

public class TrackingWeightsCommand implements Command<TrackingWeightsCommand, TrackingWeightsMessage>
{
   public double handWeight;
   public double footWeight;
   public double momentumWeight;
   public double chestWeight;
   public double pelvisOrientationWeight;
   public double privilegedWeight;
   public double privilegedConfigurationGain;
   public double privilegedMaxVelocity;

   public BodyWeights referenceWeights;

   public enum BodyWeights
   {
      STANDARD, HIGH
   };

   @Override
   public void clear()
   {
      setBodyWeights(BodyWeights.STANDARD);
   }

   @Override
   public void set(TrackingWeightsCommand other)
   {
      handWeight = other.handWeight;
      footWeight = other.footWeight;
      momentumWeight = other.momentumWeight;
      chestWeight = other.chestWeight;
      pelvisOrientationWeight = other.pelvisOrientationWeight;
      privilegedWeight = other.privilegedWeight;
      privilegedConfigurationGain = other.privilegedConfigurationGain;
      privilegedMaxVelocity = other.privilegedMaxVelocity;
   }

   @Override
   public void set(TrackingWeightsMessage message)
   {
      BodyWeights bodyWeights = message.getBodyWeights();
      setBodyWeights(bodyWeights);
   }

   @Override
   public Class<TrackingWeightsMessage> getMessageClass()
   {
      return TrackingWeightsMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   public void setBodyWeights(BodyWeights bodyWeight)
   {
      switch (bodyWeight)
      {
      case STANDARD:
         handWeight = 20.0;
         footWeight = 200.0;
         momentumWeight = 1.0;
         chestWeight = 0.02;
         pelvisOrientationWeight = 0.02;
         privilegedWeight = 1.0;
         privilegedConfigurationGain = 50.0;
         privilegedMaxVelocity = Double.POSITIVE_INFINITY;
         break;
         
      case HIGH:
         handWeight = 20.0;
         footWeight = 200.0;
         momentumWeight = 1.0;
         chestWeight = 1.00;
         pelvisOrientationWeight = 1.00;
         privilegedWeight = 1.0;
         privilegedConfigurationGain = 50.0;
         privilegedMaxVelocity = Double.POSITIVE_INFINITY;
         break;
         
      default:
         break;
      }
   }
}
