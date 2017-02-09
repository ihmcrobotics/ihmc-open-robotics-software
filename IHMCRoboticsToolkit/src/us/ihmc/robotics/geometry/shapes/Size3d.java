package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.transformables.TransformableVector3d;

/**
 * Extension of Vector3d where x, y, and z have aliases for
 * length, width, and height, respectively.
 */
class Size3d extends TransformableVector3d
{
   private static final long serialVersionUID = -6792410294569029172L;

   public Size3d()
   {
      super();
   }

   /**
   * Alias for X, Y, Z
   */
   public Size3d(double length, double width, double height)
   {
      super(length, width, height);
   }

   /**
    * Alias for X
    */
   public double getLength()
   {
      return getX();
   }

   /**
   * Alias for Y
   */
   public double getWidth()
   {
      return getY();
   }

   /**
   * Alias for Z
   */
   public double getHeight()
   {
      return getZ();
   }

   /**
   * Alias for X
   */
   public void setLength(double length)
   {
      setX(length);
   }

   /**
   * Alias for Y
   */
   public void setWidth(double width)
   {
      setY(width);
   }

   /**
   * Alias for Z
   */
   public void setHeight(double height)
   {
      setZ(height);
   }

   /**
   * Alias for X, Y, Z
   */
   public void setLengthWidthHeight(double length, double width, double height)
   {
      set(length, width, height);
   }
}
