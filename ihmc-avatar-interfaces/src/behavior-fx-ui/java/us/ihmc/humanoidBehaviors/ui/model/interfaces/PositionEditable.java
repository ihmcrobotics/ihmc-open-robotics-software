package us.ihmc.humanoidBehaviors.ui.model.interfaces;

import us.ihmc.euclid.tuple3D.Point3D;

public interface PositionEditable extends FXUIEditableGraphic
{
   void setPosition(Point3D position);
}
