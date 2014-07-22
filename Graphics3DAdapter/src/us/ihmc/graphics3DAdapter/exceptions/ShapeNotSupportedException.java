package us.ihmc.graphics3DAdapter.exceptions;

import us.ihmc.utilities.math.geometry.Shape3d;

@SuppressWarnings("serial")
public class ShapeNotSupportedException extends Exception
{
   public ShapeNotSupportedException(Shape3d shape)
   {
      super(shape.getClass().getName());
   }
}
