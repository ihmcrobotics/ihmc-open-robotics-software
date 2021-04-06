package us.ihmc.behaviors.javafx.model.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PositionEditable extends FXUIEditableGraphic
{
   void setPosition(Point3DReadOnly position);
}
