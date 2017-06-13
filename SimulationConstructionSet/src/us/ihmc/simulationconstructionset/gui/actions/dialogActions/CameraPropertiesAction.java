package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.CameraPropertiesDialogConstructor;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class CameraPropertiesAction extends SCSAction
{
   private final CameraPropertiesDialogConstructor constructor;
   private final TrackCheckBox trackCheckBox;
   private final DollyCheckBox dollyCheckBox;
   
   public CameraPropertiesAction(CameraPropertiesDialogConstructor constructor, TrackCheckBox trackCheckBox, DollyCheckBox dollyCheckBox)
   {
      super("Camera Properties...",
              "",
              KeyEvent.VK_C,
              "Open Camera Properties",
              "Open Camera Properties dialog for in-depth settings adjustment."
      );
      
      this.constructor = constructor;
      this.trackCheckBox = trackCheckBox;
      this.dollyCheckBox = dollyCheckBox;
   }

   @Override
   public void doAction()
   {
      constructor.constructCameraPropertiesDialog(trackCheckBox, dollyCheckBox);
   }
}
