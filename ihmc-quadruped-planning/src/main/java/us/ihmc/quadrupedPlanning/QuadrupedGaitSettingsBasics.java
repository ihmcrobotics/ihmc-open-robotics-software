package us.ihmc.quadrupedPlanning;

import us.ihmc.commons.MathTools;

public interface QuadrupedGaitSettingsBasics extends QuadrupedGaitSettingsReadOnly
{
   void setEndPhaseShift(double endPhaseShift);

   default void set(QuadrupedGaitSettingsReadOnly other)
   {
      setEndPhaseShift(other.getEndPhaseShift());
      setPaceSlowSettings(other.getPaceSlowSettings());
      setPaceMediumSettings(other.getPaceMediumSettings());
      setPaceFastSettings(other.getPaceFastSettings());
      setAmbleSlowSettings(other.getAmbleSlowSettings());
      setAmbleMediumSettings(other.getAmbleMediumSettings());
      setAmbleFastSettings(other.getAmbleFastSettings());
      setTrotSlowSettings(other.getTrotSlowSettings());
      setTrotMediumSettings(other.getTrotMediumSettings());
      setTrotFastSettings(other.getTrotFastSettings());
   }

   default void setPaceSlowSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getPaceSlowSettings().set(settings);
   }

   default void setPaceMediumSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getPaceMediumSettings().set(settings);
   }

   default void setPaceFastSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getPaceFastSettings().set(settings);
   }

   default void setAmbleSlowSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getAmbleSlowSettings().set(settings);
   }

   default void setAmbleMediumSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getAmbleMediumSettings().set(settings);
   }

   default void setAmbleFastSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getAmbleFastSettings().set(settings);
   }

   default void setTrotSlowSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getTrotSlowSettings().set(settings);
   }

   default void setTrotMediumSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getTrotMediumSettings().set(settings);
   }

   default void setTrotFastSettings(QuadrupedXGaitSettingsReadOnly settings)
   {
      getTrotFastSettings().set(settings);
   }

   @Override
   default QuadrupedXGaitSettingsBasics getCurrentGaitSettings(QuadrupedSpeed speed)
   {
      return getCurrentGaitSettings(speed, getEndPhaseShift());
   }

   @Override
   default QuadrupedXGaitSettingsBasics getCurrentGaitSettings(QuadrupedSpeed speed, double endPhaseShift)
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

   @Override
   QuadrupedXGaitSettingsBasics getPaceSlowSettings();
   @Override
   QuadrupedXGaitSettingsBasics getPaceMediumSettings();
   @Override
   QuadrupedXGaitSettingsBasics getPaceFastSettings();

   @Override
   QuadrupedXGaitSettingsBasics getAmbleSlowSettings();
   @Override
   QuadrupedXGaitSettingsBasics getAmbleMediumSettings();
   @Override
   QuadrupedXGaitSettingsBasics getAmbleFastSettings();

   @Override
   QuadrupedXGaitSettingsBasics getTrotSlowSettings();
   @Override
   QuadrupedXGaitSettingsBasics getTrotMediumSettings();
   @Override
   QuadrupedXGaitSettingsBasics getTrotFastSettings();


}
