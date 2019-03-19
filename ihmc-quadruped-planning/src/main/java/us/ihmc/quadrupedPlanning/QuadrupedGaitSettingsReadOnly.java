package us.ihmc.quadrupedPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;

public interface QuadrupedGaitSettingsReadOnly
{
   /**
    * Nominal phase shift between front and hind steps (in degrees, 0: pace, 90: amble, 180: trot).
    */
   double getEndPhaseShift();



   QuadrupedXGaitSettingsReadOnly getPaceSlowSettings();
   QuadrupedXGaitSettingsReadOnly getPaceMediumSettings();
   QuadrupedXGaitSettingsReadOnly getPaceFastSettings();

   QuadrupedXGaitSettingsReadOnly getAmbleSlowSettings();
   QuadrupedXGaitSettingsReadOnly getAmbleMediumSettings();
   QuadrupedXGaitSettingsReadOnly getAmbleFastSettings();

   QuadrupedXGaitSettingsReadOnly getTrotSlowSettings();
   QuadrupedXGaitSettingsReadOnly getTrotMediumSettings();
   QuadrupedXGaitSettingsReadOnly getTrotFastSettings();

   default QuadrupedSpeed getSpeedForSettings(double speed)
   {
      return getSpeedForSettings(speed, getEndPhaseShift());
   }

   default QuadrupedSpeed getSpeedForSettings(double speed, double endPhaseShift)
   {
      endPhaseShift = MathTools.clamp(endPhaseShift, 0.0, 180.0);
      if (endPhaseShift < 45.0)
      {
         if (speed < getPaceSlowSettings().getMaxSpeed())
            return QuadrupedSpeed.SLOW;
         else if (speed < getPaceMediumSettings().getMaxSpeed())
            return QuadrupedSpeed.MEDIUM;
         else
            return QuadrupedSpeed.FAST;
      }
      else if (endPhaseShift < 135.0)
      {
         if (speed < getAmbleSlowSettings().getMaxSpeed())
            return QuadrupedSpeed.SLOW;
         else if (speed < getAmbleMediumSettings().getMaxSpeed())
            return QuadrupedSpeed.MEDIUM;
         else
            return QuadrupedSpeed.FAST;
      }
      else
      {
         if (speed < getTrotSlowSettings().getMaxSpeed())
            return QuadrupedSpeed.SLOW;
         else if (speed < getTrotMediumSettings().getMaxSpeed())
            return QuadrupedSpeed.MEDIUM;
         else
            return QuadrupedSpeed.FAST;
      }
   }

   default QuadrupedXGaitSettingsReadOnly getCurrentGaitSettings(QuadrupedSpeed speed)
   {
      return getCurrentGaitSettings(speed, getEndPhaseShift());
   }

   default QuadrupedXGaitSettingsReadOnly getCurrentGaitSettings(QuadrupedSpeed speed, double endPhaseShift)
   {
      endPhaseShift = MathTools.clamp(endPhaseShift, 0.0, 180.0);
      if (endPhaseShift < 45.0)
      {
         if (speed == QuadrupedSpeed.SLOW)
            return getPaceSlowSettings();
         else if (speed == QuadrupedSpeed.MEDIUM)
            return getPaceMediumSettings();
         else
            return getPaceFastSettings();
      }
      else if (endPhaseShift < 135.0)
      {
         if (speed == QuadrupedSpeed.SLOW)
            return getAmbleSlowSettings();
         else if (speed == QuadrupedSpeed.MEDIUM)
            return getAmbleMediumSettings();
         else
            return getAmbleFastSettings();
      }
      else
      {
         if (speed == QuadrupedSpeed.SLOW)
            return getTrotSlowSettings();
         else if (speed == QuadrupedSpeed.MEDIUM)
            return getTrotMediumSettings();
         else
            return getTrotFastSettings();
      }
   }

   /**
    * Nominal x offset between front and hind feet (in meters).
    *
    * !! WARNING !! generates garbage
    */
   default double getStanceLength(QuadrupedSpeed speed)
   {
      return getStanceLength(speed, getEndPhaseShift());
   }

   /**
    * Nominal x offset between front and hind feet (in meters).
    *
    * !! WARNING !! generates garbage
    */
   default double getStanceLength(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedXGaitSettingsReadOnly::getStanceLength);
   }

   /**
    * Nominal y offset between left and right feet (in meters).
    *
    * !! WARNING !! generates garbage
    */
   default double getStanceWidth(QuadrupedSpeed speed)
   {
      return getStanceWidth(speed, getEndPhaseShift());
   }

   /**
    * Nominal y offset between left and right feet (in meters).
    *
    * !! WARNING !! generates garbage
    */
   default double getStanceWidth(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedXGaitSettingsReadOnly::getStanceWidth);
   }

   /**
    * Ground clearance for each step (in meters).
    *
    * !! WARNING !! generates garbage
    */
   default double getStepGroundClearance(QuadrupedSpeed speed)
   {
      return getStepGroundClearance(speed, getEndPhaseShift());
   }

   /**
    * Ground clearance for each step (in meters).
    *
    * !! WARNING !! generates garbage
    */
   default double getStepGroundClearance(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedXGaitSettingsReadOnly::getStepGroundClearance);
   }

   /**
    * Time duration of each swing phase (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getStepDuration(QuadrupedSpeed speed)
   {
      return getStepDuration(speed, getEndPhaseShift());
   }

   /**
    * Time duration of each swing phase (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getStepDuration(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedXGaitSettingsReadOnly::getStepDuration);
   }

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getEndDoubleSupportDuration(QuadrupedSpeed speed)
   {
      return getEndDoubleSupportDuration(speed, getEndPhaseShift());
   }

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    *
    * !! WARNING !! generates garbage
    */
   default double getEndDoubleSupportDuration(QuadrupedSpeed speed, double endPhaseShift)
   {
      return interpolate(speed, endPhaseShift, this, QuadrupedXGaitSettingsReadOnly::getEndDoubleSupportDuration);
   }

   static double interpolate(QuadrupedSpeed speed, double endPhaseShift, QuadrupedGaitSettingsReadOnly gaitSettings, SettingProvider settingProvider)
   {
      endPhaseShift = MathTools.clamp(endPhaseShift, 0.0, 180.0);
      switch (speed)
      {
      case SLOW:
         if (endPhaseShift < 90.0)
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getPaceSlowSettings()),
                                                        settingProvider.getSetting(gaitSettings.getAmbleSlowSettings()),
                                                        endPhaseShift / 90.0);
         else
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getAmbleSlowSettings()),
                                                        settingProvider.getSetting(gaitSettings.getTrotSlowSettings()),
                                                        (endPhaseShift - 90.0) / 90.0);
      case MEDIUM:
         if (endPhaseShift < 90.0)
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getPaceMediumSettings()),
                                                        settingProvider.getSetting(gaitSettings.getAmbleMediumSettings()),
                                                        endPhaseShift / 90.0);
         else
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getAmbleMediumSettings()),
                                                        settingProvider.getSetting(gaitSettings.getTrotMediumSettings()),
                                                        (endPhaseShift - 90.0) / 90.0);
      default:
         if (endPhaseShift < 90.0)
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getPaceFastSettings()),
                                                        settingProvider.getSetting(gaitSettings.getAmbleFastSettings()),
                                                        endPhaseShift / 90.0);
         else
            return InterpolationTools.linearInterpolate(settingProvider.getSetting(gaitSettings.getAmbleFastSettings()),
                                                        settingProvider.getSetting(gaitSettings.getTrotFastSettings()),
                                                        (endPhaseShift - 90.0) / 90.0);
      }
   }

}
