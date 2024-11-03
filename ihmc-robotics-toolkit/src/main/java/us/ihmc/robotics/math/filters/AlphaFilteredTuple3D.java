package us.ihmc.robotics.math.filters;

import org.apache.commons.lang3.NotImplementedException;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class AlphaFilteredTuple3D implements Tuple3DBasics, Settable<Tuple3DBasics>
{
   private double alpha;

   private double x = Double.NaN;
   private double y = Double.NaN;
   private double z = Double.NaN;

   private boolean resetX = false;
   private boolean resetY = false;
   private boolean resetZ = false;

   public AlphaFilteredTuple3D()
   {
      this.alpha = 0.0;
   }

   public AlphaFilteredTuple3D(double alpha)
   {
      this.alpha = alpha;
   }

   public AlphaFilteredTuple3D(double x, double y, double z, double alpha)
   {
      this.alpha = alpha;
      beginReset();
      set(x, y, z);
   }

   public AlphaFilteredTuple3D(Tuple3DReadOnly other, double alpha)
   {
      this.alpha = alpha;
      beginReset();
      set(other);
   }

   public void beginReset()
   {
      resetX = true;
      resetY = true;
      resetZ = true;
   }

   public void endReset()
   {
      resetX = false;
      resetY = false;
      resetZ = false;
   }

   public void reset(Tuple3DReadOnly other)
   {
      beginReset();
      set(other);
   }

   @Override
   public void setX(double x)
   {
      if (resetX || Double.isNaN(this.x))
      {
         this.x = x;
         resetX = false;
      }
      else
      {
         this.x = EuclidCoreTools.interpolate(x, this.x, alpha);
      }
   }

   @Override
   public void setY(double y)
   {
      if (resetY || Double.isNaN(this.y))
      {
         this.y = y;
         resetY = false;
      }
      else
      {
         this.y = EuclidCoreTools.interpolate(y, this.y, alpha);
      }
   }

   @Override
   public void setZ(double z)
   {
      if (resetZ || Double.isNaN(this.z))
      {
         this.z = z;
         resetZ = false;
      }
      else
      {
         this.z = EuclidCoreTools.interpolate(z, this.z, alpha);
      }
   }

   public void setAlpha(double alpha)
   {
      this.alpha = alpha;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      throw new NotImplementedException("Sorry mate, " + getClass().getSimpleName() + " doesn't implement this method.");
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      throw new NotImplementedException("Sorry mate, " + getClass().getSimpleName() + " doesn't implement this method.");
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return epsilonEquals(geometry, epsilon);
   }

   @Override
   public void set(Tuple3DBasics other)
   {
      beginReset();
      interpolate(other, this, alpha);
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
