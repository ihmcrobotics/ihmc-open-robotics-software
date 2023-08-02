package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class AlphaFilteredRigidBodyTransform extends RigidBodyTransform
{
   private boolean hasBeenCalled = false;
   /** Also think of it as max rate */
   private double alpha = 0.0;
   private final RigidBodyTransform previousFiltered = new RigidBodyTransform();

   public void update(RigidBodyTransform measured)
   {
      if (hasBeenCalled)
      {
         previousFiltered.set(this);
         interpolate(measured, previousFiltered, alpha);
      }
      else
      {
         hasBeenCalled = true;
         set(measured);
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
