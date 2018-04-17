package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.*;
import java.awt.event.ActionEvent;

import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;
import us.ihmc.yoVariables.dataBuffer.ToggleKeyPointModeCommandExecutor;
import us.ihmc.yoVariables.dataBuffer.ToggleKeyPointModeCommandListener;

import javax.swing.*;

public class ToggleKeyPointModeAction extends AbstractAction implements ToggleKeyPointModeCommandListener
{
   private static final long serialVersionUID = 1500047530568017379L;
   
   private final String iconFilename = "icons/ToggleKeyMode.png";
   private final String altFilename = "icons/ToggleKeyModePressed.png";
   
   private Image iconImage = AbstractActionTools.loadActionImageUsingInputStream(this, iconFilename);
   private Image altImage = AbstractActionTools.loadActionImageUsingInputStream(this, altFilename);
   
   private ImageIcon icon = new ImageIcon(iconImage);

   private ToggleKeyPointModeCommandExecutor executor;

   public ToggleKeyPointModeAction(ToggleKeyPointModeCommandExecutor executor)
   {
      super("Toggle Key Mode");
      this.executor = executor;

      this.putValue(Action.SMALL_ICON, icon);
      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_F));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");

      executor.registerToggleKeyPointModeCommandListener(this);
   }
      

   @Override
   public void updateKeyPointModeStatus()
   {
      if (executor.isKeyPointModeToggled())
      {
         icon.setImage(iconImage);
      }
      else
      {
         icon.setImage(altImage);
      }
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.toggleKeyPointMode();
   }

   @Override
   public void closeAndDispose()
   {
      iconImage = null;
      altImage = null;
      icon = null;
      if (executor != null) executor.closeAndDispose();
      executor = null;
   }
}
