package us.ihmc.pathPlanning.visibilityGraphs.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class YawProperty extends ParametersProperty<Quaternion>
{
   private final DoubleField yaw = new DoubleField(Quaternion::getYaw, (p, v) -> p.setYawPitchRoll(v, 0.0, 0.0));

   public YawProperty(Object bean, String name)
   {
      this(bean, name, 0.0);
   }

   public YawProperty(Object bean, String name, double initialValue)
   {
      super(bean, name, new Quaternion(initialValue, 0.0, 0.0));
   }

   public void bindBidirectionalYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yaw);
   }


   @Override
   protected Quaternion getValueCopy(Quaternion valueToCopy)
   {
      return new Quaternion(valueToCopy);
   }

}
