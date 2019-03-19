package us.ihmc.quadrupedPlanning;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoQuadrupedGaitSettings implements QuadrupedGaitSettingsBasics
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble endPhaseShift = new YoDouble("endPhaseShift", registry);

   private final QuadrupedXGaitSettingsBasics paceSlowSettings;
   private final QuadrupedXGaitSettingsBasics paceMediumSettings;
   private final QuadrupedXGaitSettingsBasics paceFastSettings;
   private final QuadrupedXGaitSettingsBasics ambleSlowSettings;
   private final QuadrupedXGaitSettingsBasics ambleMediumSettings;
   private final QuadrupedXGaitSettingsBasics ambleFastSettings;
   private final QuadrupedXGaitSettingsBasics trotSlowSettings;
   private final QuadrupedXGaitSettingsBasics trotMediumSettings;
   private final QuadrupedXGaitSettingsBasics trotFastSettings;

   public YoQuadrupedGaitSettings(QuadrupedGaitSettingsReadOnly defaultSettings, YoVariableRegistry parentRegistry)
   {
      paceSlowSettings = new YoQuadrupedXGaitSettings("paceSlow", defaultSettings.getPaceSlowSettings(), registry);
      paceMediumSettings = new YoQuadrupedXGaitSettings("paceMedium", defaultSettings.getPaceMediumSettings(), registry);
      paceFastSettings = new YoQuadrupedXGaitSettings("paceFast", defaultSettings.getPaceMediumSettings(), registry);
      ambleSlowSettings = new YoQuadrupedXGaitSettings("ambleSlow", defaultSettings.getAmbleSlowSettings(), registry);
      ambleMediumSettings = new YoQuadrupedXGaitSettings("ambleMedium", defaultSettings.getAmbleSlowSettings(), registry);
      ambleFastSettings = new YoQuadrupedXGaitSettings("ambleFast", defaultSettings.getAmbleSlowSettings(), registry);
      trotSlowSettings = new YoQuadrupedXGaitSettings("trotSlow", defaultSettings.getAmbleSlowSettings(), registry);
      trotMediumSettings = new YoQuadrupedXGaitSettings("trotMedium", defaultSettings.getAmbleSlowSettings(), registry);
      trotFastSettings = new YoQuadrupedXGaitSettings("trotFast", defaultSettings.getAmbleSlowSettings(), registry);

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
   public void setEndPhaseShift(double endPhaseShift)
   {
      this.endPhaseShift.set(endPhaseShift);
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getPaceSlowSettings()
   {
      return paceSlowSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getPaceMediumSettings()
   {
      return paceMediumSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getPaceFastSettings()
   {
      return paceFastSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getAmbleSlowSettings()
   {
      return ambleSlowSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getAmbleMediumSettings()
   {
      return ambleMediumSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getAmbleFastSettings()
   {
      return ambleFastSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getTrotSlowSettings()
   {
      return trotSlowSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getTrotMediumSettings()
   {
      return trotMediumSettings;
   }

   /** {@inheritDoc} */
   @Override
   public QuadrupedXGaitSettingsBasics getTrotFastSettings()
   {
      return trotFastSettings;
   }
}
