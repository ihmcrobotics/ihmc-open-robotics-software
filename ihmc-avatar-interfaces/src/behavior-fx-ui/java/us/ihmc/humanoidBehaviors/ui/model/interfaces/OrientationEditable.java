package us.ihmc.humanoidBehaviors.ui.model.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;

public interface OrientationEditable extends FXUIEditableGraphic
{
   void setOrientation(Orientation3DReadOnly orientationPoint);
}
