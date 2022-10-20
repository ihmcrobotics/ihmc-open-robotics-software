package us.ihmc.rdx.input.editor;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PositionEditable extends GDXUIEditableGraphic
{
   void setPosition(Point3DReadOnly position);
}
