package us.ihmc.pathPlanning.visibilityGraphs.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class Point3DProperty extends ParametersProperty<Point3D>
{
   private final DoubleField x = new DoubleField(Point3D::getX, (p, v) -> p.setX(v));
   private final DoubleField y = new DoubleField(Point3D::getY, (p, v) -> p.setY(v));
   private final DoubleField z = new DoubleField(Point3D::getZ, (p, v) -> p.setZ(v));

   public Point3DProperty(Object bean, String name)
   {
      this(bean, name, new Point3D());
   }

   public Point3DProperty(Object bean, String name, Point3D initialValue)
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
   
   public void bindBidirectionalZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, z);
   }

   @Override
   protected Point3D getValueCopy(Point3D valueToCopy)
   {
      return new Point3D(valueToCopy);
   }

}
