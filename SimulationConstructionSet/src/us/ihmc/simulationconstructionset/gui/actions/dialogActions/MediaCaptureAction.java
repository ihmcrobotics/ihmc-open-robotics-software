package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class MediaCaptureAction extends SCSAction
{
   private MediaCaptureDialogConstructor constructor;

   public MediaCaptureAction(MediaCaptureDialogConstructor constructor)
   {
      super("Media Capture...",
              "icons/Capture.png",
              KeyEvent.VK_M,
              "Capture Media",
              "Capture media to a file."
      );

      this.constructor = constructor;
   }

   public void createVideoFromFile(File file)
   {
      constructor.createVideo(file);
   }

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }
}
