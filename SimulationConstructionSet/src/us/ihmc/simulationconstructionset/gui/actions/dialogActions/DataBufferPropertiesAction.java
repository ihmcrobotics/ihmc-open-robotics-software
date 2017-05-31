package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.DataBufferPropertiesDialogConstructor;

public class DataBufferPropertiesAction extends SCSAction
{
   private static final long serialVersionUID = -3128885419773084408L;
   private DataBufferPropertiesDialogConstructor constructor;

   public DataBufferPropertiesAction(DataBufferPropertiesDialogConstructor constructor)
   {
      super("Data Buffer Properties...",
              "",
              KeyEvent.VK_B,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.constructor = constructor;
   }

   public void doAction()
   {
      constructor.constructDialog();
   }

   public void closeAndDispose()
   {
      if (constructor != null)
      {
         constructor.closeAndDispose();
         constructor = null;
      }
   }
}
