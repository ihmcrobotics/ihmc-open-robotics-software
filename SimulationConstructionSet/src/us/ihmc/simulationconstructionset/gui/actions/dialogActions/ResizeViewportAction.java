package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.ResizeViewportDialogConstructor;

public class ResizeViewportAction extends AbstractAction
{
   private static final long serialVersionUID = 8843463124373810864L;
   private ResizeViewportDialogConstructor constructor;

   public ResizeViewportAction(ResizeViewportDialogConstructor constructor)
   {
      super("Resize Viewport");
      this.constructor = constructor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_V));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
     constructor.constructDialog();
   }
}
