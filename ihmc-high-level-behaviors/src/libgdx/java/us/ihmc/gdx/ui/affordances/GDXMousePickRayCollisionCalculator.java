package us.ihmc.gdx.ui.affordances;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;

public interface GDXMousePickRayCollisionCalculator
{
   /**
    * Allows for arbitrary mouse pick ray collision calculation.
    *
    * @param mousePickRay
    * @return The closest intersection point or NaN if no collision.
    */
   public double calculateClosestCollision(Line3DReadOnly mousePickRay);
}
