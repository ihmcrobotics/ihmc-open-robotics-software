package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;

public interface QuadrupedXGaitSettingsReadOnly
{
   double getMaxSpeed();

   /**
    * Nominal x offset between front and hind feet (in meters).
    */
   double getStanceLength();

   /**
    * Nominal y offset between left and right feet (in meters).
    */
   double getStanceWidth();

   /**
    * Ground clearance for each step (in meters).
    */
   double getStepGroundClearance();

   /**
    * Time duration of each swing phase (in seconds).
    */
   double getStepDuration();

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    */
   double getEndDoubleSupportDuration();

   QuadrupedXGaitSettingsPacket getAsPacket();
}
