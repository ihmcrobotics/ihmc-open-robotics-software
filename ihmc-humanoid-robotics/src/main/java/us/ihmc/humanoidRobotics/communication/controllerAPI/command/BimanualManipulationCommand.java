package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.BimanualManipulationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class BimanualManipulationCommand implements Command<BimanualManipulationCommand, BimanualManipulationMessage>
{
   private boolean disable;
   private double objectMass = 0.0;
   private double squeezeForce = 0.0;
   private double initializeDuration = 0.0;
   private double trackingErrorThreshold = 0.0;

   @Override
   public void clear()
   {
      disable = false;
      objectMass = 0.0;
      squeezeForce = 0.0;
      initializeDuration = 0.0;
      trackingErrorThreshold = 0.0;
   }

   @Override
   public void setFromMessage(BimanualManipulationMessage message)
   {
      this.disable = message.getDisable();
      this.objectMass = message.getObjectMass();
      this.squeezeForce = message.getSqueezeForce();
      this.initializeDuration = message.getInitializeDuration();
      this.trackingErrorThreshold = message.getAcceptableTrackingError();
   }

   @Override
   public Class<BimanualManipulationMessage> getMessageClass()
   {
      return BimanualManipulationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return disable || (objectMass > 0.0 && squeezeForce >= 0.0);
   }

   @Override
   public long getSequenceId()
   {
      return 0;
   }

   @Override
   public void set(BimanualManipulationCommand other)
   {
      this.disable = other.disable;
      this.objectMass = other.objectMass;
      this.squeezeForce = other.squeezeForce;
      this.initializeDuration = other.initializeDuration;
      this.trackingErrorThreshold = other.trackingErrorThreshold;
   }

   public boolean isDisableRequested()
   {
      return disable;
   }

   public double getObjectMass()
   {
      return objectMass;
   }

   public double getSqueezeForce()
   {
      return squeezeForce;
   }

   public double getInitializeDuration()
   {
      return initializeDuration;
   }

   public double getTrackingErrorThreshold()
   {
      return trackingErrorThreshold;
   }
}
