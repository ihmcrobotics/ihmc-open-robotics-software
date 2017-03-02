package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.Container;

import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.gui.ActiveCameraHolder;
import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;
import us.ihmc.simulationconstructionset.gui.dialogs.CameraPropertiesDialog;

public class CameraPropertiesDialogGenerator implements CameraPropertiesDialogConstructor
{
   private ActiveCameraHolder cameraHolder;
   private Container parentContainer;
   private JFrame frame;

   public CameraPropertiesDialogGenerator(ActiveCameraHolder cameraHolder, Container parentContainer,
           JFrame frame)
   {
      this.parentContainer = parentContainer;
      this.frame = frame;

      this.cameraHolder = cameraHolder;
   }

   @Override
   public void constructCameraPropertiesDialog(TrackCheckBox trackCheckBox, DollyCheckBox dollyCheckBox)
   {
      new CameraPropertiesDialog(parentContainer, frame, trackCheckBox, dollyCheckBox, cameraHolder);
   }

   public void closeAndDispose()
   {
      cameraHolder = null;
      parentContainer = null;
      frame = null;
   }
}
