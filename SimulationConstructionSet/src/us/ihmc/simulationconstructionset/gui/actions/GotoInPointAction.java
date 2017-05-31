package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.GotoInPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class GotoInPointAction extends SCSAction
{
   private static final long serialVersionUID = -9080927302316708389L;

   private final GotoInPointCommandExecutor executor;

   public GotoInPointAction(GotoInPointCommandExecutor executor)
   {
      super("Goto In Point",
              "icons/YoGoInPoint24_2.gif",
              KeyEvent.VK_I,
              "Goto In Point",
              "Goto In Point"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.gotoInPoint();
   }
}
