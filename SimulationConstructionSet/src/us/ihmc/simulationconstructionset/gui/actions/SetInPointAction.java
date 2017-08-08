package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class SetInPointAction extends AbstractAction
{
   private static final long serialVersionUID = 1396893923616884444L;

   private SetInPointCommandExecutor executor;

   public SetInPointAction(SetInPointCommandExecutor executor)
   {
      super("Set In Point");
      this.executor = executor;

      String iconFilename = "icons/YoSetInPoint24.gif";
      int shortKey = KeyEvent.VK_N;
      String longDescription = "Set In Point";
      String shortDescription = "Set In Point";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.setInPoint();
   }
}
