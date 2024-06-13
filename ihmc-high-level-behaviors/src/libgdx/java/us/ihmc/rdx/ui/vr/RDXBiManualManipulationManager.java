package us.ihmc.rdx.ui.vr;

import controller_msgs.msg.dds.BimanualManipulationMessage;

public class RDXBiManualManipulationManager
{

   // TO DO : FIELDS NEED TO BE SET FROM THE UI

   private boolean enableBiManualManipulation = true;
   /**
    * Mass of the object being manipulated
    */
   public double object_mass_ = -1.0;
   /**
    * Lateral squeeze force while manipulating the object. Should approximately be squeeze_force = 0.5 * g * object_mass / friction_coefficient
    */
   public double squeeze_force_ = -1.0;
   /**
    * The squeeze and mass compensation forces ramp up over this duration (if negative a default value is used)
    */
   public double initialize_duration_ = -1.0;
   /**
    * Tracking error - if the distance between the hands varies from their initial distance by more than this value, the manipulation manager stops
    */
   public double acceptable_tracking_error_ = -1.0;

   public void toggleBiManualManipulationMode()
   {
      enableBiManualManipulation = !enableBiManualManipulation;
   }

   public BimanualManipulationMessage getBiManualManipulationMessage()
   {
      BimanualManipulationMessage message = new BimanualManipulationMessage();
      message.setDisable(enableBiManualManipulation);
      return message;
   }

   public void setEnableBiManualManipulationMode(boolean enableBiManualManipulation)
   {
      this.enableBiManualManipulation = enableBiManualManipulation;
   }

   public boolean getEnableBiManualManipulationMode()
   {
      return enableBiManualManipulation;
   }

}
