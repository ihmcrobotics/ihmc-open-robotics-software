package us.ihmc.gdx.input.editor;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface PoseEditable extends PositionEditable, OrientationEditable
{
   Point3DBasics getPosition();
}
