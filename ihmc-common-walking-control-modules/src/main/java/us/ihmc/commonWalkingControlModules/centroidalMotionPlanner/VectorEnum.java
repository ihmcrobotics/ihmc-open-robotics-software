package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;

public class VectorEnum<F> implements Settable<VectorEnum<F>>, Clearable
{
   private F x;
   private F y;
   private F z;

   public F getX()
   {
      return x;
   }

   public F getY()
   {
      return y;
   }

   public F getZ()
   {
      return z;
   }

   public F getElement(Axis axis)
   {
      switch (axis)
      {
      case X:
         return getX();
      case Y:
         return getY();
      case Z:
         return getZ();
      default:
         throw new RuntimeException("Invalid axis specified");
      }
   }

   @Override
   public boolean containsNaN()
   {
      return (x == null || y == null || z == null);
   }

   @Override
   public void setToNaN()
   {
      setToNull();
   }

   @Override
   public void setToZero()
   {
      setToNull();
   }

   public void setToNull()
   {
      x = null;
      y = null;
      z = null;
   }

   @Override
   public void set(VectorEnum<F> other)
   {
      this.x = other.x;
      this.y = other.y;
      this.z = other.z;
   }

   public void setX(F xVal)
   {
      this.x = xVal;
   }

   public void setY(F yVal)
   {
      this.y = yVal;
   }

   public void setZ(F zVal)
   {
      this.z = zVal;
   }

   public void set(F xVal, F yVal, F zVal)
   {
      setX(xVal);
      setY(yVal);
      setZ(zVal);
   }

   public void setElement(Axis axis, F elementVal)
   {
      switch (axis)
      {
      case X:
         setX(elementVal);
         break;
      case Y:
         setY(elementVal);
         break;
      case Z:
         setZ(elementVal);
         break;
      default:
         throw new RuntimeException("Invalid axis specified");
      }
   }
}