package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.AddCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class AddCameraKeyAction extends SCSAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private final AddCameraKeyCommandExecutor executor;

   public AddCameraKeyAction(AddCameraKeyCommandExecutor listener)
   {
      super("Add Camera Key",
              "",
              KeyEvent.VK_A,
              "Adds camera key.",
              "Adds a camera key so that the camera will automatically transition to the position it is in and at the time indicated on the graph when this button is pressed."
      );

      this.executor = listener;

   }

   public void doAction()
   {
      executor.addCameraKey();
   }
}