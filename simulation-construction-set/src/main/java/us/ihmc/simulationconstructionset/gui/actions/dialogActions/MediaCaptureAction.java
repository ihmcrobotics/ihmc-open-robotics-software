package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogConstructor;

public class MediaCaptureAction extends AbstractAction
{
   private static final long serialVersionUID = 772924943132573130L;
   private MediaCaptureDialogConstructor constructor;

   public MediaCaptureAction(MediaCaptureDialogConstructor constructor)
   {
      super("Media Capture...");
      this.constructor = constructor;

      String iconFilename = "icons/Video24.gif";
      int shortKey = KeyEvent.VK_M;
      String longDescription = "Capture media to a file.";
      String shortDescription = "Capture Media";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   public void createVideoFromFile(File file)
   {
      constructor.createVideo(file);
   }

   @Override
   public void actionPerformed(ActionEvent event)
   {
      constructor.constructDialog();
   }
}
