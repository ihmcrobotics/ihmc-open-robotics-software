package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogConstructor;

public class MediaCaptureAction extends SCSAction
{
   private static final long serialVersionUID = 772924943132573130L;
   private MediaCaptureDialogConstructor constructor;

   public MediaCaptureAction(MediaCaptureDialogConstructor constructor)
   {
      super("Media Capture...",
              "icons/Video24.gif",
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

   public void doAction()
   {
      constructor.constructDialog();
   }
}
