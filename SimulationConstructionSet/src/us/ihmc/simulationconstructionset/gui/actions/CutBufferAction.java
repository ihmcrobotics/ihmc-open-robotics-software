package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CutBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class CutBufferAction extends SCSAction
{
   private static final long serialVersionUID = 5039888169851613916L;
   private CutBufferCommandExecutor executor;

   public CutBufferAction(CutBufferCommandExecutor executor)
   {
      super("Cut Buffer Between In/Out",
              "",
              KeyEvent.VK_C,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.cutBuffer();
   }
}
