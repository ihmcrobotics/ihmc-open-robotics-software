package us.ihmc.atlas.joystickBasedStepping;

import javafx.beans.property.Property;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class Vector2DProperty extends ParametersProperty<Vector2D>
{
   private final DoubleField x = new DoubleField(Vector2D::getX, (p, v) -> p.setX(v));
   private final DoubleField y = new DoubleField(Vector2D::getY, (p, v) -> p.setY(v));

   public Vector2DProperty(Object bean, String name)
   {
      this(bean, name, new Vector2D());
   }

   public Vector2DProperty(Object bean, String name, Vector2D initialValue)
   {
      super(bean, name, initialValue);
   }

   public void bindBidirectionalX(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, x);
   }

   public void bindBidirectionalY(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, y);
   }

   @Override
   protected Vector2D getValueCopy(Vector2D valueToCopy)
   {
      return new Vector2D(valueToCopy);
   }
}
