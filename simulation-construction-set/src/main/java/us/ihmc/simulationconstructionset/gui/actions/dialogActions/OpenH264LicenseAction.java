package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.codecs.loader.OpenH264Downloader;

public class OpenH264LicenseAction extends AbstractAction
{
   private static final long serialVersionUID = -4079097951250188360L;

   public OpenH264LicenseAction()
   {
      super("Show OpenH264 License...");
      

      this.putValue(Action.LONG_DESCRIPTION, "Display OpenH264 License");
      this.putValue(Action.SHORT_DESCRIPTION, "OpenH264 License");

   }
   
   @Override
   public void actionPerformed(ActionEvent e)
   {
      OpenH264Downloader.showAboutCiscoDialog();
   }

}
