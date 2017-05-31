package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class SetInPointAction extends SCSAction
{
   private static final long serialVersionUID = 1396893923616884444L;

   private SetInPointCommandExecutor executor;

   public SetInPointAction(SetInPointCommandExecutor executor)
   {
      super("Set In Point",
              "icons/YoSetInPoint24.gif",
              KeyEvent.VK_N,
              "Set In Point",
              "Set In Point"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.setInPoint();
   }
}
