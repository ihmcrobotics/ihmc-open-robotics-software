package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ResizeViewportDialogConstructor;

public class ResizeViewportAction extends SCSAction
{
   private static final long serialVersionUID = 8843463124373810864L;
   private ResizeViewportDialogConstructor constructor;

   public ResizeViewportAction(ResizeViewportDialogConstructor constructor)
   {
      super("Resize Viewport",
              "",
              KeyEvent.VK_V,
              "Short Description",
              "Long Description"
      );

      this.constructor = constructor;
   }

   public void doAction()
   {
     constructor.constructDialog();
   }
}
