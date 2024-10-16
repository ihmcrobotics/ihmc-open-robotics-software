package us.ihmc.yoVariables.euclid.filters;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreTools;
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

   public AlphaFilteredTuple2D(Tuple2DReadOnly initialValue, DoubleProvider alpha)
   {
      this.alpha = alpha;
      reset(initialValue);
   }

   public void reset()
   {
      resetX = true;
      resetY = true;
   }

   public void reset(Tuple2DReadOnly other)
   {
      reset(other.getX(), other.getY());
   }

   public void reset(double x, double y)
   {
      reset();
      set(x, y);
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
         this.x = EuclidCoreTools.interpolate(x, this.x, alpha.getValue());
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
         this.y = EuclidCoreTools.interpolate(y, this.y, alpha.getValue());
      }
   }

   @Override
   public boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return epsilonEquals(geometry, epsilon);
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
}
