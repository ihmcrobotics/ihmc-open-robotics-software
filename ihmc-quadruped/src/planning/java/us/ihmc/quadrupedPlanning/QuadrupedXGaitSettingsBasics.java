package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;

public interface QuadrupedXGaitSettingsBasics extends QuadrupedXGaitSettingsReadOnly
{
   void setEndPhaseShift(double endPhaseShift);

   void setQuadrupedSpeed(QuadrupedSpeed quadrupedSpeed);

   void setStanceLength(double stanceLength);

   void setStanceWidth(double stanceWidth);

   void setMaxHorizontalSpeedFraction(double fraction);

   void setMaxYawSpeedFraction(double fraction);

   void setStepGroundClearance(double stepGroundClearance);

   default void set(QuadrupedXGaitSettingsReadOnly other)
   {
      setEndPhaseShift(other.getEndPhaseShift());
      setQuadrupedSpeed(other.getQuadrupedSpeed());
      setStanceLength(other.getStanceLength());
      setStanceWidth(other.getStanceWidth());
      setStepGroundClearance(other.getStepGroundClearance());
      setMaxHorizontalSpeedFraction(other.getMaxHorizontalSpeedFraction());
      setMaxYawSpeedFraction(other.getMaxYawSpeedFraction());

      setPaceSlowSettings(other.getPaceSlowTimings());
      setPaceMediumSettings(other.getPaceMediumTimings());
      setPaceFastSettings(other.getPaceFastTimings());
      setAmbleSlowSettings(other.getAmbleSlowTimings());
      setAmbleMediumSettings(other.getAmbleMediumTimings());
      setAmbleFastSettings(other.getAmbleFastTimings());
      setTrotSlowSettings(other.getTrotSlowTimings());
      setTrotMediumSettings(other.getTrotMediumTimings());
      setTrotFastSettings(other.getTrotFastTimings());
   }

   default void set(QuadrupedXGaitSettingsPacket packet)
   {
      setEndPhaseShift(packet.getEndPhaseShift());
      setQuadrupedSpeed(QuadrupedSpeed.fromByte(packet.getQuadrupedSpeed()));
      setStanceLength(packet.getStanceLength());
      setStanceWidth(packet.getStanceWidth());
      setStepGroundClearance(packet.getStepGroundClearance());
      setMaxHorizontalSpeedFraction(packet.getMaxHorizontalSpeedFraction());
      setMaxYawSpeedFraction(packet.getMaxYawSpeedFraction());

      getPaceSlowTimings().set(packet.getPaceSlowSettingsPacket());
      getPaceMediumTimings().set(packet.getPaceMediumSettingsPacket());
      getPaceFastTimings().set(packet.getPaceFastSettingsPacket());
      getAmbleSlowTimings().set(packet.getAmbleSlowSettingsPacket());
      getAmbleMediumTimings().set(packet.getAmbleMediumSettingsPacket());
      getAmbleFastTimings().set(packet.getAmbleFastSettingsPacket());
      getTrotSlowTimings().set(packet.getTrotSlowSettingsPacket());
      getTrotMediumTimings().set(packet.getTrotMediumSettingsPacket());
      getTrotFastTimings().set(packet.getTrotFastSettingsPacket());
   }

   default void setPaceSlowSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getPaceSlowTimings().set(settings);
   }

   default void setPaceMediumSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getPaceMediumTimings().set(settings);
   }

   default void setPaceFastSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getPaceFastTimings().set(settings);
   }

   default void setAmbleSlowSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getAmbleSlowTimings().set(settings);
   }

   default void setAmbleMediumSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getAmbleMediumTimings().set(settings);
   }

   default void setAmbleFastSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getAmbleFastTimings().set(settings);
   }

   default void setTrotSlowSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getTrotSlowTimings().set(settings);
   }

   default void setTrotMediumSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getTrotMediumTimings().set(settings);
   }

   default void setTrotFastSettings(QuadrupedGaitTimingsReadOnly settings)
   {
      getTrotFastTimings().set(settings);
   }

   @Override
   QuadrupedGaitTimingsBasics getPaceSlowTimings();
   @Override
   QuadrupedGaitTimingsBasics getPaceMediumTimings();
   @Override
   QuadrupedGaitTimingsBasics getPaceFastTimings();

   @Override
   QuadrupedGaitTimingsBasics getAmbleSlowTimings();
   @Override
   QuadrupedGaitTimingsBasics getAmbleMediumTimings();
   @Override
   QuadrupedGaitTimingsBasics getAmbleFastTimings();

   @Override
   QuadrupedGaitTimingsBasics getTrotSlowTimings();
   @Override
   QuadrupedGaitTimingsBasics getTrotMediumTimings();
   @Override
   QuadrupedGaitTimingsBasics getTrotFastTimings();


}
