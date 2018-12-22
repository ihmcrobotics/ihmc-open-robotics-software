package us.ihmc.quadrupedUI;

import javafx.beans.property.Property;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class XGaitSettingsProperty extends ParametersProperty<QuadrupedXGaitSettings>
{
   private final DoubleField stanceLength = new DoubleField(QuadrupedXGaitSettings::getStanceLength, QuadrupedXGaitSettings::setStanceLength);
   private final DoubleField stanceWidth = new DoubleField(QuadrupedXGaitSettings::getStanceWidth, QuadrupedXGaitSettings::setStanceWidth);
   private final DoubleField stepGroundClearance = new DoubleField(QuadrupedXGaitSettings::getStepGroundClearance, QuadrupedXGaitSettings::setStepGroundClearance);
   private final DoubleField stepDuration = new DoubleField(QuadrupedXGaitSettings::getStepDuration, QuadrupedXGaitSettings::setStepDuration);
   private final DoubleField endDoubleSupportDuration = new DoubleField(QuadrupedXGaitSettings::getEndDoubleSupportDuration, QuadrupedXGaitSettings::setEndDoubleSupportDuration);
   private final DoubleField endPhaseShift = new DoubleField(QuadrupedXGaitSettings::getEndPhaseShift, QuadrupedXGaitSettings::setEndPhaseShift);


   public XGaitSettingsProperty(Object bean, String name, QuadrupedXGaitSettings xGaitSettings)
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

}
