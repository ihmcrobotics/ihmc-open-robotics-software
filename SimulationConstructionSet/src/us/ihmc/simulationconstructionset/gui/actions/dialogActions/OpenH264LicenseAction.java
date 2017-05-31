package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.codecs.loader.OpenH264Downloader;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class OpenH264LicenseAction extends SCSAction
{
   private static final long serialVersionUID = -4079097951250188360L;

   public OpenH264LicenseAction()
   {
      super("Show OpenH264 License...",
              "",
              KeyEvent.VK_UNDEFINED,
              "OpenH264 License",
              "Display OpenH264 License"
      );
   }

   public void doAction()
   {
      OpenH264Downloader.showAboutCiscoDialog();
   }

}
