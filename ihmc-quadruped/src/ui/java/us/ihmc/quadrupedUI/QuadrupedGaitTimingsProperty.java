package us.ihmc.quadrupedUI;

import javafx.beans.property.Property;
import us.ihmc.quadrupedPlanning.QuadrupedGaitTimings;
import us.ihmc.quadrupedPlanning.QuadrupedGaitTimingsReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class QuadrupedGaitTimingsProperty extends ParametersProperty<QuadrupedGaitTimings>
{
   private final DoubleField maxSpeed = new DoubleField(QuadrupedGaitTimings::getMaxSpeed, QuadrupedGaitTimings::setMaxSpeed);
   private final DoubleField stepDuration = new DoubleField(QuadrupedGaitTimings::getStepDuration, QuadrupedGaitTimings::setStepDuration);
   private final DoubleField endDoubleSupportDuration = new DoubleField(QuadrupedGaitTimings::getEndDoubleSupportDuration, QuadrupedGaitTimings::setEndDoubleSupportDuration);

   public QuadrupedGaitTimingsProperty(Object bean, String name, QuadrupedGaitTimingsReadOnly gaitTimings)
   {
      super(bean, name, new QuadrupedGaitTimings(gaitTimings));
   }

   @Override
   protected QuadrupedGaitTimings getValueCopy(QuadrupedGaitTimings valueToCopy)
   {
      return new QuadrupedGaitTimings(valueToCopy);
   }

   public void bidirectionalBindMaxSpeed(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxSpeed);
   }

   public void bidirectionalBindStepDuration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepDuration);
   }

   public void bidirectionalBindEndDoubleSupportDuration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, endDoubleSupportDuration);
   }
}
