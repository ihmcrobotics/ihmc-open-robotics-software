package us.ihmc.graphicsDescription.exceptions;

import us.ihmc.euclid.geometry.Shape3D;

@SuppressWarnings("serial")
public class ShapeNotSupportedException extends Exception
{
   public ShapeNotSupportedException(Shape3D shape)
   {
      super(shape.getClass().getName());
   }
}
