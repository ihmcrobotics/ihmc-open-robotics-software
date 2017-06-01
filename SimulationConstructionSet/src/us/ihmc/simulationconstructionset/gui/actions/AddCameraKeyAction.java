package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.AddCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class AddCameraKeyAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      executor.addCameraKey();
   }
}