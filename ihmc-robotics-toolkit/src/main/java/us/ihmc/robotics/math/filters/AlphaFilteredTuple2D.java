package us.ihmc.robotics.math.filters;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class AlphaFilteredTuple2D implements Tuple2DBasics
{
   private final DoubleProvider alpha;

   private boolean resetX;
   private boolean resetY;

   private double x;
   private double y;

   public AlphaFilteredTuple2D(DoubleProvider alpha)
   {
      this.alpha = alpha;
      reset();
   }

   public AlphaFilteredTuple2D(Tuple2DReadOnly other, DoubleProvider alpha)
   {
      this.alpha = alpha;
      reset(other);
   }

   public void reset()
   {
      resetX = true;
      resetY = true;
   }

   public void reset(Tuple2DReadOnly other)
   {
      resetX = true;
      resetY = true;
      set(other);
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
   public void setX(double x)
   {
      if (resetX)
      {
         this.x = x;
         resetX = false;
      }
      else
      {
         this.x = alpha.getValue() * this.x + (1.0 - alpha.getValue()) * x;
      }
   }

   @Override
   public void setY(double y)
   {
      if (resetY)
      {
         this.y = y;
         resetY = false;
      }
      else
      {
         this.y = alpha.getValue() * this.y + (1.0 - alpha.getValue()) * y;
      }
   }

   @Override
   public void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      throw new NotImplementedException("Not supported by " + getClass().getSimpleName() + ".");
   }

   @Override
   public void applyInverseTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      throw new NotImplementedException("Not supported by " + getClass().getSimpleName() + ".");
   }

   @Override
   public boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return epsilonEquals(geometry, epsilon);
   }
}
