package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class AboutAction extends SCSAction
{
   private final AboutDialogConstructor constructor;

   public AboutAction(AboutDialogConstructor constructor)
   {
      super("About...",
              "",
              KeyEvent.VK_A,
              "About",
              "Display About Information"
      );

      this.constructor = constructor;
   }

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }

}
