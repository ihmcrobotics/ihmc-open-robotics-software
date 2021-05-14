package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;

public interface QuadrupedXGaitSettingsReadOnly
{
   QuadrupedSpeed getQuadrupedSpeed();

   /**
    * Nominal phase shift between front and hind steps (in degrees, 0: pace, 90: amble, 180: trot).
    */
   double getEndPhaseShift();

   /**
    * Nominal x offset between front and hind feet (in meters).
    */
   double getStanceLength();

   /**
    * Nominal y offset between left and right feet (in meters).
    */
   double getStanceWidth();

   /**
    * Fractional multiplier of the max speed returned by {@link #getMaxSpeed()} to determine the maximum horizontal speed.
    */
   default double getMaxHorizontalSpeedFraction()
   {
      return 0.5;
   }

   /**
    * Fractional multiplier of the max speed returned by {@link #getMaxSpeed()} to determine the maximum yaw speed.
    */
   default double getMaxYawSpeedFraction()
   {
      return 0.75;
   }

   /**
    * Ground clearance for each step (in meters).
    */
   double getStepGroundClearance();

   QuadrupedGaitTimingsReadOnly getPaceSlowTimings();

   QuadrupedGaitTimingsReadOnly getPaceMediumTimings();

   QuadrupedGaitTimingsReadOnly getPaceFastTimings();

   QuadrupedGaitTimingsReadOnly getAmbleSlowTimings();

   QuadrupedGaitTimingsReadOnly getAmbleMediumTimings();

   QuadrupedGaitTimingsReadOnly getAmbleFastTimings();

   QuadrupedGaitTimingsReadOnly getTrotSlowTimings();

   QuadrupedGaitTimingsReadOnly getTrotMediumTimings();

   QuadrupedGaitTimingsReadOnly getTrotFastTimings();

   /**
    * Time duration of each swing phase (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getStepDuration()
   {
      return getStepDuration(getQuadrupedSpeed(), getEndPhaseShift());
   }


   /**
    * Time duration of each swing phase (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getStepDuration(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedGaitTimingsReadOnly::getStepDuration);
   }

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getEndDoubleSupportDuration()
   {
      return getEndDoubleSupportDuration(getQuadrupedSpeed(), getEndPhaseShift());
   }

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getEndDoubleSupportDuration(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedGaitTimingsReadOnly::getEndDoubleSupportDuration);
   }

   default double getMaxSpeed()
   {
      return getMaxSpeed(getQuadrupedSpeed(), getEndPhaseShift());
   }

   default double getMaxSpeed(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedGaitTimingsReadOnly::getMaxSpeed);
   }

   static double interpolate(QuadrupedSpeed speed, double endPhaseShift, QuadrupedXGaitSettingsReadOnly gaitSettings, SettingProvider settingProvider)
   {
      endPhaseShift = MathTools.clamp(endPhaseShift, 0.0, 180.0);
      switch (speed)
      {
      case SLOW:
         if (endPhaseShift < 90.0)
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getPaceSlowTimings()),
                                                        settingProvider.getSetting(gaitSettings.getAmbleSlowTimings()), endPhaseShift / 90.0);
         else
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getAmbleSlowTimings()),
                                                        settingProvider.getSetting(gaitSettings.getTrotSlowTimings()), (endPhaseShift - 90.0) / 90.0);
      case MEDIUM:
         if (endPhaseShift < 90.0)
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getPaceMediumTimings()),
                                                        settingProvider.getSetting(gaitSettings.getAmbleMediumTimings()), endPhaseShift / 90.0);
         else
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getAmbleMediumTimings()),
                                                        settingProvider.getSetting(gaitSettings.getTrotMediumTimings()), (endPhaseShift - 90.0) / 90.0);
      default:
         if (endPhaseShift < 90.0)
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getPaceFastTimings()),
                                                        settingProvider.getSetting(gaitSettings.getAmbleFastTimings()), endPhaseShift / 90.0);
         else
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getAmbleFastTimings()),
                                                        settingProvider.getSetting(gaitSettings.getTrotFastTimings()), (endPhaseShift - 90.0) / 90.0);
      }
   }

   default QuadrupedXGaitSettingsPacket getAsPacket()
   {
      QuadrupedXGaitSettingsPacket packet = new QuadrupedXGaitSettingsPacket();
      packet.setQuadrupedSpeed(getQuadrupedSpeed().toByte());
      packet.setEndPhaseShift(getEndPhaseShift());
      packet.setStanceLength(getStanceLength());
      packet.setStanceWidth(getStanceWidth());
      packet.setStepGroundClearance(getStepGroundClearance());
      packet.setMaxHorizontalSpeedFraction(getMaxHorizontalSpeedFraction());
      packet.setMaxYawSpeedFraction(getMaxYawSpeedFraction());
      packet.getPaceSlowSettingsPacket().set(getPaceSlowTimings().getAsPacket());
      packet.getPaceMediumSettingsPacket().set(getPaceMediumTimings().getAsPacket());
      packet.getPaceFastSettingsPacket().set(getPaceFastTimings().getAsPacket());
      packet.getAmbleSlowSettingsPacket().set(getAmbleSlowTimings().getAsPacket());
      packet.getAmbleMediumSettingsPacket().set(getAmbleMediumTimings().getAsPacket());
      packet.getAmbleFastSettingsPacket().set(getAmbleFastTimings().getAsPacket());
      packet.getTrotSlowSettingsPacket().set(getTrotSlowTimings().getAsPacket());
      packet.getTrotMediumSettingsPacket().set(getTrotMediumTimings().getAsPacket());
      packet.getTrotFastSettingsPacket().set(getTrotFastTimings().getAsPacket());

      return packet;
   }
}
