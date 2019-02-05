package us.ihmc.quadrupedUI;

import javafx.beans.property.Property;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class QuadrupedXGaitSettingsProperty extends ParametersProperty<QuadrupedXGaitSettings>
{
   private final DoubleField stanceLength = new DoubleField(QuadrupedXGaitSettings::getStanceLength, QuadrupedXGaitSettings::setStanceLength);
   private final DoubleField stanceWidth = new DoubleField(QuadrupedXGaitSettings::getStanceWidth, QuadrupedXGaitSettings::setStanceWidth);
   private final DoubleField stepGroundClearance = new DoubleField(QuadrupedXGaitSettings::getStepGroundClearance, QuadrupedXGaitSettings::setStepGroundClearance);
   private final DoubleField stepDuration = new DoubleField(QuadrupedXGaitSettings::getStepDuration, QuadrupedXGaitSettings::setStepDuration);
   private final DoubleField endDoubleSupportDuration = new DoubleField(QuadrupedXGaitSettings::getEndDoubleSupportDuration, QuadrupedXGaitSettings::setEndDoubleSupportDuration);
   private final DoubleField endPhaseShift = new DoubleField(QuadrupedXGaitSettings::getEndPhaseShift, QuadrupedXGaitSettings::setEndPhaseShift);


   public QuadrupedXGaitSettingsProperty(Object bean, String name, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(bean, name, new QuadrupedXGaitSettings(xGaitSettings));
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

   public void bidirectionalBindStepDuration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepDuration);
   }

   public void bidirectionalBindEndDoubleSupportDuration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, endDoubleSupportDuration);
   }

   public void bidirectionalBindEndPhaseShift(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, endPhaseShift);
   }
}
