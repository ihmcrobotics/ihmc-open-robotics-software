package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.net.URL;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ImageIcon;

import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;

public class AddKeyPointAction extends AbstractAction
{
   private static final long serialVersionUID = -2830335620118067620L;
   private URL iconURL = AddKeyPointAction.class.getClassLoader().getResource("icons/setKey.gif");
   private ImageIcon icon = new ImageIcon(iconURL);
   private AddKeyPointCommandExecutor executor;

   public AddKeyPointAction(AddKeyPointCommandExecutor executor)
   {
      super("Add Key Point");
      this.executor = executor;
      
      this.putValue(Action.SMALL_ICON, icon);
      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_F));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.addKeyPoint();
   }
}
