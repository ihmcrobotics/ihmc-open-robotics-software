package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.net.URL;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ImageIcon;

import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class AddKeyPointAction extends SCSAction
{
   private static final long serialVersionUID = -2830335620118067620L;
   private AddKeyPointCommandExecutor executor;

   public AddKeyPointAction(AddKeyPointCommandExecutor executor)
   {
      super("Add Key Point",
              "icons/setKey.gif",
              KeyEvent.VK_F,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.addKeyPoint();
   }
}
