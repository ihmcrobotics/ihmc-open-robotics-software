package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class AddKeyPointAction extends AbstractAction
{
   private static final long serialVersionUID = -2830335620118067620L;
   private AddKeyPointCommandExecutor executor;

   public AddKeyPointAction(AddKeyPointCommandExecutor executor)
   {
      super("Add Key Point");
      this.executor = executor;

      String iconFilename = "icons/NewKey.png";
      int shortKey = KeyEvent.VK_F;
      String longDescription = "Long Description";
      String shortDescription = "Short Description";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.addKeyPoint();
   }
}
