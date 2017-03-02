package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.net.URL;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ImageIcon;

import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;

public class ToggleKeyPointModeAction extends AbstractAction implements ToggleKeyPointModeCommandListener
{
   private static final long serialVersionUID = 1500047530568017379L;
   private URL iconURL = ToggleKeyPointModeAction.class.getClassLoader().getResource("icons/toggleKey.gif");
   private ImageIcon icon = new ImageIcon(iconURL);

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
         iconURL = ToggleKeyPointModeAction.class.getClassLoader().getResource("icons/toggleKeyPressed.gif");
         ImageIcon tmp = new ImageIcon(iconURL);
         icon.setImage(tmp.getImage());
      }
      else
      {
         iconURL = ToggleKeyPointModeAction.class.getClassLoader().getResource("icons/toggleKey.gif");
         ImageIcon tmp = new ImageIcon(iconURL);
         icon.setImage(tmp.getImage());
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
      iconURL = null;
      icon = null;
      if (executor != null) executor.closeAndDispose();
      executor = null;
   }


}
