package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.CameraPropertiesDialogConstructor;

public class CameraPropertiesAction extends SCSAction
{
   private static final long serialVersionUID = 2728147089395054885L;

   private final CameraPropertiesDialogConstructor constructor;
   private final TrackCheckBox trackCheckBox;
   private final DollyCheckBox dollyCheckBox;
   
   public CameraPropertiesAction(CameraPropertiesDialogConstructor constructor, TrackCheckBox trackCheckBox, DollyCheckBox dollyCheckBox)
   {
      super("Camera Properties...",
              "",
              KeyEvent.VK_C,
              "Short Description", // TODO
              "Long Description" // TODO
      );
      
      this.constructor = constructor;
      this.trackCheckBox = trackCheckBox;
      this.dollyCheckBox = dollyCheckBox;
   }

   public void doAction()
   {
      constructor.constructCameraPropertiesDialog(trackCheckBox, dollyCheckBox);
   }
}
