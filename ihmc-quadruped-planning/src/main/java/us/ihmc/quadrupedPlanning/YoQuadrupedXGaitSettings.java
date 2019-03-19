package us.ihmc.quadrupedPlanning;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoQuadrupedXGaitSettings implements QuadrupedXGaitSettingsBasics
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble endPhaseShift = new YoDouble("endPhaseShift", registry);
   private final YoDouble stanceLength = new YoDouble("stanceLength", registry);
   private final YoDouble stanceWidth = new YoDouble("stanceWidth", registry);
   private final YoDouble stepGroundClearance = new YoDouble("stepGroundClearance", registry);
   private final YoEnum<QuadrupedSpeed> quadrupedSpeed = YoEnum.create("quadrupedSpeed", QuadrupedSpeed.class, registry);

   private final QuadrupedGaitTimingsBasics paceSlowSettings;
   private final QuadrupedGaitTimingsBasics paceMediumSettings;
   private final QuadrupedGaitTimingsBasics paceFastSettings;
   private final QuadrupedGaitTimingsBasics ambleSlowSettings;
   private final QuadrupedGaitTimingsBasics ambleMediumSettings;
   private final QuadrupedGaitTimingsBasics ambleFastSettings;
   private final QuadrupedGaitTimingsBasics trotSlowSettings;
   private final QuadrupedGaitTimingsBasics trotMediumSettings;
   private final QuadrupedGaitTimingsBasics trotFastSettings;

   public YoQuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly defaultSettings, YoVariableRegistry parentRegistry)
   {
      paceSlowSettings = new YoQuadrupedGaitTimings("paceSlow", defaultSettings.getPaceSlowTimings(), registry);
      paceMediumSettings = new YoQuadrupedGaitTimings("paceMedium", defaultSettings.getPaceMediumTimings(), registry);
      paceFastSettings = new YoQuadrupedGaitTimings("paceFast", defaultSettings.getPaceMediumTimings(), registry);
      ambleSlowSettings = new YoQuadrupedGaitTimings("ambleSlow", defaultSettings.getAmbleSlowTimings(), registry);
      ambleMediumSettings = new YoQuadrupedGaitTimings("ambleMedium", defaultSettings.getAmbleSlowTimings(), registry);
      ambleFastSettings = new YoQuadrupedGaitTimings("ambleFast", defaultSettings.getAmbleSlowTimings(), registry);
      trotSlowSettings = new YoQuadrupedGaitTimings("trotSlow", defaultSettings.getAmbleSlowTimings(), registry);
      trotMediumSettings = new YoQuadrupedGaitTimings("trotMedium", defaultSettings.getAmbleSlowTimings(), registry);
      trotFastSettings = new YoQuadrupedGaitTimings("trotFast", defaultSettings.getAmbleSlowTimings(), registry);

      set(defaultSettings);

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public double getEndPhaseShift()
   {
      return endPhaseShift.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStanceLength()
   {
      return stanceLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStanceWidth()
   {
      return stanceWidth.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStepGroundClearance()
   {
      return stepGroundClearance.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedSpeed getQuadrupedSpeed()
   {
      return quadrupedSpeed.getEnumValue();
   }

   /** {@inheritDoc} */
   @Override
   public void setEndPhaseShift(double endPhaseShift)
   {
      this.endPhaseShift.set(endPhaseShift);
   }

   /** {@inheritDoc} */
   @Override
   public void setStanceLength(double stanceLength)
   {
      this.stanceLength.set(stanceLength);
   }

   /** {@inheritDoc} */
   @Override
   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth.set(stanceWidth);
   }

   /** {@inheritDoc} */
   @Override
   public void setStepGroundClearance(double stepGroundClearance)
   {
      this.stepGroundClearance.set(stepGroundClearance);
   }

   /** {@inheritDoc} */
   @Override
   public void setQuadrupedSpeed(QuadrupedSpeed quadrupedSpeed)
   {
      this.quadrupedSpeed.set(quadrupedSpeed);
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getPaceSlowTimings()
   {
      return paceSlowSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getPaceMediumTimings()
   {
      return paceMediumSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getPaceFastTimings()
   {
      return paceFastSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getAmbleSlowTimings()
   {
      return ambleSlowSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getAmbleMediumTimings()
   {
      return ambleMediumSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getAmbleFastTimings()
   {
      return ambleFastSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getTrotSlowTimings()
   {
      return trotSlowSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getTrotMediumTimings()
   {
      return trotMediumSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedGaitTimingsBasics getTrotFastTimings()
   {
      return trotFastSettings;
   }
}
