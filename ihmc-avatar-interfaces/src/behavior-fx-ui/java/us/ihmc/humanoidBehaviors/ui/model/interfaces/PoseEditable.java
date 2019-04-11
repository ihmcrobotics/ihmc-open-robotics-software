package us.ihmc.humanoidBehaviors.ui.model.interfaces;

import us.ihmc.euclid.tuple3D.Point3D;

public interface PoseEditable extends PositionEditable, OrientationEditable
{
   Point3D getPosition();
}
