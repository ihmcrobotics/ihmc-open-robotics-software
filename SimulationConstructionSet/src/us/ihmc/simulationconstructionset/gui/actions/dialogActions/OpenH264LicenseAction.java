package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.codecs.loader.OpenH264Downloader;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class OpenH264LicenseAction extends SCSAction
{
   public OpenH264LicenseAction()
   {
      super("Show OpenH264 License...",
              "",
              KeyEvent.VK_UNDEFINED,
              "OpenH264 License",
              "Display OpenH264 License"
      );
   }

   @Override
   public void doAction()
   {
      OpenH264Downloader.showAboutCiscoDialog();
   }
}
