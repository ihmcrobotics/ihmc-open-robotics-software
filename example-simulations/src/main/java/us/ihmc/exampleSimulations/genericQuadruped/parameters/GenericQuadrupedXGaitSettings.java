package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedPlanning.*;

public class GenericQuadrupedXGaitSettings implements QuadrupedXGaitSettingsReadOnly
{
   private final QuadrupedGaitTimingsBasics ambleSlowTimings = new QuadrupedGaitTimings();
   private final QuadrupedGaitTimingsBasics ambleMediumTimings = new QuadrupedGaitTimings();
   private final QuadrupedGaitTimingsBasics ambleFastTimings = new QuadrupedGaitTimings();

   private final QuadrupedGaitTimingsBasics paceSlowTimings = new QuadrupedGaitTimings();
   private final QuadrupedGaitTimingsBasics paceMediumTimings = new QuadrupedGaitTimings();
   private final QuadrupedGaitTimingsBasics paceFastTimings = new QuadrupedGaitTimings();

   private final QuadrupedGaitTimingsBasics trotSlowTimings = new QuadrupedGaitTimings();
   private final QuadrupedGaitTimingsBasics trotMediumTimings = new QuadrupedGaitTimings();
   private final QuadrupedGaitTimingsBasics trotFastTimings = new QuadrupedGaitTimings();

   public GenericQuadrupedXGaitSettings()
   {
      ambleSlowTimings.setStepDuration(0.4);
      ambleSlowTimings.setEndDoubleSupportDuration(0.3);
      ambleSlowTimings.setMaxSpeed(0.4);

      ambleMediumTimings.setStepDuration(0.35);
      ambleMediumTimings.setEndDoubleSupportDuration(0.15);
      ambleMediumTimings.setMaxSpeed(0.8);

      ambleFastTimings.setStepDuration(0.25);
      ambleFastTimings.setEndDoubleSupportDuration(0.05);
      ambleFastTimings.setMaxSpeed(1.25);

      paceSlowTimings.setStepDuration(0.25);
      paceSlowTimings.setEndDoubleSupportDuration(0.25);
      paceSlowTimings.setMaxSpeed(0.3);

      paceMediumTimings.setStepDuration(0.25);
      paceMediumTimings.setEndDoubleSupportDuration(0.1);
      paceMediumTimings.setMaxSpeed(0.6);

      paceFastTimings.setStepDuration(0.25);
      paceFastTimings.setEndDoubleSupportDuration(0.001);
      paceFastTimings.setMaxSpeed(0.9);

      trotSlowTimings.setStepDuration(0.35);
      trotSlowTimings.setEndDoubleSupportDuration(0.25);
      trotSlowTimings.setMaxSpeed(0.5);

      trotMediumTimings.setStepDuration(0.3);
      trotMediumTimings.setEndDoubleSupportDuration(0.15);
      trotMediumTimings.setMaxSpeed(1.25);

      trotFastTimings.setStepDuration(0.25);
      trotFastTimings.setEndDoubleSupportDuration(0.05);
      trotFastTimings.setMaxSpeed(1.75);
   }

   @Override
   public double getStanceLength()
   {
      return 1.1;
   }

   @Override
   public double getStanceWidth()
   {
      return 0.2;
   }

   @Override
   public double getStepGroundClearance()
   {
      return 0.1;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getPaceSlowTimings()
   {
      return paceSlowTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getPaceMediumTimings()
   {
      return paceMediumTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getPaceFastTimings()
   {
      return paceFastTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getAmbleSlowTimings()
   {
      return ambleSlowTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getAmbleMediumTimings()
   {
      return ambleMediumTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getAmbleFastTimings()
   {
      return ambleFastTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getTrotSlowTimings()
   {
      return trotSlowTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getTrotMediumTimings()
   {
      return trotMediumTimings;
   }

   @Override
   public QuadrupedGaitTimingsReadOnly getTrotFastTimings()
   {
      return trotFastTimings;
   }

   @Override
   public QuadrupedSpeed getQuadrupedSpeed()
   {
      return QuadrupedSpeed.MEDIUM;
   }

   @Override
   public double getEndPhaseShift()
   {
      return 90.0;
   }

}
