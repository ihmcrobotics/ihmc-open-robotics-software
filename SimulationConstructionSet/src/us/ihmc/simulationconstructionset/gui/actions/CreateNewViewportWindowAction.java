package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CreateNewViewportWindowCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class CreateNewViewportWindowAction extends SCSAction
{
   private static final long serialVersionUID = 1890952078365483337L;
   private CreateNewViewportWindowCommandExecutor executor;

   public CreateNewViewportWindowAction(CreateNewViewportWindowCommandExecutor executor)
   {
      super("New Viewport Window",
              "",
              KeyEvent.VK_V,
              "Creates a new Viewport Window.",
              "Creates a new Viewport Window for showing 3D Graphics in SCS."
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.createNewViewportWindow();
   }
}
