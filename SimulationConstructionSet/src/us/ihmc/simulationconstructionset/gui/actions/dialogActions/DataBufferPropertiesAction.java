package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.DataBufferPropertiesDialogConstructor;

public class DataBufferPropertiesAction extends AbstractAction
{
   private static final long serialVersionUID = -3128885419773084408L;
   private DataBufferPropertiesDialogConstructor constructor;

   public DataBufferPropertiesAction(DataBufferPropertiesDialogConstructor constructor)
   {
      super("Data Buffer Properties...");
      this.constructor = constructor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_B));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
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
