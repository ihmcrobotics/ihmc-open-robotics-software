package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class AlphaFilteredRigidBodyTransform extends RigidBodyTransform
{
   /** Also think of it as max rate */
   private double alpha = 0.0;
   private final RigidBodyTransform previousFiltered = new RigidBodyTransform();
   {
      previousFiltered.setToNaN();
   }

   public void update(RigidBodyTransform measured)
   {
      if (previousFiltered.containsNaN())
      {
         set(measured);
      }
      else
      {
         previousFiltered.set(this);
         interpolate(measured, previousFiltered, alpha);
      }
   }

   public double getAlpha()
   {
      return alpha;
   }

   public void setAlpha(double alpha)
   {
      this.alpha = alpha;
   }
}
