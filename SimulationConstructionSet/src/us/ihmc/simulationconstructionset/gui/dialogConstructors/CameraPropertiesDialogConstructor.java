package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;

public interface CameraPropertiesDialogConstructor
{
   public abstract void constructCameraPropertiesDialog(TrackCheckBox trackCheckBox, DollyCheckBox dollyCheckBox);
}
