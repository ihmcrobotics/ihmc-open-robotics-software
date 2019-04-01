package us.ihmc.humanoidBehaviors.ui.model.interfaces;

import us.ihmc.euclid.tuple3D.Point3D;

public interface OrientationEditable extends FXUIEditableGraphic
{
   void setOrientation(Point3D orientationPoint);
}
