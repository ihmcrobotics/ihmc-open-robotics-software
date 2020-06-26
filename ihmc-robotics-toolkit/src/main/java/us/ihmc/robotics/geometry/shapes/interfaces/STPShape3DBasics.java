package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;

public interface STPShape3DBasics extends STPShape3DReadOnly, Shape3DBasics
{
   void setMargins(double minimumMargin, double maximumMargin);
}
