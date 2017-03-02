package us.ihmc.graphicsDescription.exceptions;

import us.ihmc.robotics.geometry.shapes.Shape3d;

@SuppressWarnings("serial")
public class ShapeNotSupportedException extends Exception
{
   public ShapeNotSupportedException(Shape3d shape)
   {
      super(shape.getClass().getName());
   }
}
