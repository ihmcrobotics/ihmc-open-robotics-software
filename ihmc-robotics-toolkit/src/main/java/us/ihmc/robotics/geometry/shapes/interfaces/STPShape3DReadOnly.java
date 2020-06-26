package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;

public interface STPShape3DReadOnly extends Shape3DReadOnly
{
   double getMinimumMargin();

   double getMaximumMargin();

   double getSmallRadius();

   double getLargeRadius();
}
