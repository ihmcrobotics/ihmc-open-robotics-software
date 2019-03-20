package us.ihmc.quadrupedUI;

import javafx.beans.property.Property;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class QuadrupedXGaitSettingsProperty extends ParametersProperty<QuadrupedXGaitSettings>
{
   private final DoubleField stanceLength = new DoubleField(QuadrupedXGaitSettings::getStanceLength, QuadrupedXGaitSettings::setStanceLength);
   private final DoubleField stanceWidth = new DoubleField(QuadrupedXGaitSettings::getStanceWidth, QuadrupedXGaitSettings::setStanceWidth);
   private final DoubleField stepGroundClearance = new DoubleField(QuadrupedXGaitSettings::getStepGroundClearance, QuadrupedXGaitSettings::setStepGroundClearance);
   private final DoubleField endPhaseShift = new DoubleField(QuadrupedXGaitSettings::getEndPhaseShift, QuadrupedXGaitSettings::setEndPhaseShift);
   private final EnumField<QuadrupedSpeed> quadrupedSpeed = new EnumField<QuadrupedSpeed>(QuadrupedXGaitSettings::getQuadrupedSpeed, QuadrupedXGaitSettings::setQuadrupedSpeed);

   private final QuadrupedGaitTimingsProperty paceSlowProperty;
   private final QuadrupedGaitTimingsProperty paceMediumProperty;
   private final QuadrupedGaitTimingsProperty paceFastProperty;
   private final QuadrupedGaitTimingsProperty ambleSlowProperty;
   private final QuadrupedGaitTimingsProperty ambleMediumProperty;
   private final QuadrupedGaitTimingsProperty ambleFastProperty;
   private final QuadrupedGaitTimingsProperty trotSlowProperty;
   private final QuadrupedGaitTimingsProperty trotMediumProperty;
   private final QuadrupedGaitTimingsProperty trotFastProperty;

   public QuadrupedXGaitSettingsProperty(Object bean, String name, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(bean, name, new QuadrupedXGaitSettings(xGaitSettings));

      paceSlowProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getPaceSlowTimings());
      paceMediumProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getPaceMediumTimings());
      paceFastProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getPaceFastTimings());
      ambleSlowProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getAmbleSlowTimings());
      ambleMediumProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getAmbleMediumTimings());
      ambleFastProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getAmbleFastTimings());
      trotSlowProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getTrotSlowTimings());
      trotMediumProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getTrotMediumTimings());
      trotFastProperty = new QuadrupedGaitTimingsProperty(bean, name, xGaitSettings.getTrotFastTimings());
   }

   @Override
   protected QuadrupedXGaitSettings getValueCopy(QuadrupedXGaitSettings valueToCopy)
   {
      return new QuadrupedXGaitSettings(valueToCopy);
   }

   public void bidirectionalBindStanceLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stanceLength);
   }

   public void bidirectionalBindStanceWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stanceWidth);
   }

   public void bidirectionalBindStepGroundClearance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepGroundClearance);
   }

   public void bidirectionalBindEndPhaseShift(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, endPhaseShift);
   }

   public void bidirectionalBindQuadrupedSpeed(Property<QuadrupedSpeed> property)
   {
      bindFieldBidirectionalToEnumProperty(property, quadrupedSpeed);
   }

   public void bidirectionalBindAmbleSlowStepDuration(Property<? extends Number> property)
   {
      ambleSlowProperty.bidirectionalBindStepDuration(property);
   }

   public void bidirectionalBindAmbleSlowEndDoubleSupportDuration(Property<? extends Number> property)
   {
      ambleSlowProperty.bidirectionalBindEndDoubleSupportDuration(property);
   }

   public void bidirectionalBindAmbleSlowMaxSpeed(Property<? extends Number> property)
   {
      ambleSlowProperty.bidirectionalBindMaxSpeed(property);
   }

   public void bidirectionalBindAmbleMediumStepDuration(Property<? extends Number> property)
   {
      ambleMediumProperty.bidirectionalBindStepDuration(property);
   }

   public void bidirectionalBindAmbleMediumEndDoubleSupportDuration(Property<? extends Number> property)
   {
      ambleMediumProperty.bidirectionalBindEndDoubleSupportDuration(property);
   }

   public void bidirectionalBindAmbleMediumMaxSpeed(Property<? extends Number> property)
   {
      ambleMediumProperty.bidirectionalBindMaxSpeed(property);
   }

   public void bidirectionalBindAmbleFastStepDuration(Property<? extends Number> property)
   {
      ambleFastProperty.bidirectionalBindStepDuration(property);
   }

   public void bidirectionalBindAmbleFastEndDoubleSupportDuration(Property<? extends Number> property)
   {
      ambleFastProperty.bidirectionalBindEndDoubleSupportDuration(property);
   }

   public void bidirectionalBindAmbleFastMaxSpeed(Property<? extends Number> property)
   {
      ambleFastProperty.bidirectionalBindMaxSpeed(property);
   }

   public void bidirectionalBindTrotSlowStepDuration(Property<? extends Number> property)
   {
      trotSlowProperty.bidirectionalBindStepDuration(property);
   }

   public void bidirectionalBindTrotSlowEndDoubleSupportDuration(Property<? extends Number> property)
   {
      trotSlowProperty.bidirectionalBindEndDoubleSupportDuration(property);
   }

   public void bidirectionalBindTrotSlowMaxSpeed(Property<? extends Number> property)
   {
      trotSlowProperty.bidirectionalBindMaxSpeed(property);
   }

   public void bidirectionalBindTrotMediumStepDuration(Property<? extends Number> property)
   {
      trotMediumProperty.bidirectionalBindStepDuration(property);
   }

   public void bidirectionalBindTrotMediumEndDoubleSupportDuration(Property<? extends Number> property)
   {
      trotMediumProperty.bidirectionalBindEndDoubleSupportDuration(property);
   }

   public void bidirectionalBindTrotMediumMaxSpeed(Property<? extends Number> property)
   {
      trotMediumProperty.bidirectionalBindMaxSpeed(property);
   }

   public void bidirectionalBindTrotFastStepDuration(Property<? extends Number> property)
   {
      trotFastProperty.bidirectionalBindStepDuration(property);
   }

   public void bidirectionalBindTrotFastEndDoubleSupportDuration(Property<? extends Number> property)
   {
      trotFastProperty.bidirectionalBindEndDoubleSupportDuration(property);
   }

   public void bidirectionalBindTrotFastMaxSpeed(Property<? extends Number> property)
   {
      trotFastProperty.bidirectionalBindMaxSpeed(property);
   }
}
