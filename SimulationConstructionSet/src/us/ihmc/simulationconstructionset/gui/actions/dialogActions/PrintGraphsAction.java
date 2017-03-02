package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogConstructor;

public class PrintGraphsAction extends AbstractAction
{
   private static final long serialVersionUID = -4403022630360741566L;
   private PrintGraphsDialogConstructor constructor;

   public PrintGraphsAction(PrintGraphsDialogConstructor constructor)
   {
      super("Print Graphs");
      this.constructor = constructor;

      String iconFilename = "icons/Print24.gif";
      int shortKey = KeyEvent.VK_P;
      String longDescription = "Print Graphs";
      String shortDescription = "Print Graphs";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   public void closeAndDispose()
   {
      constructor.closeAndDispose();
      constructor = null;
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      constructor.constructDialog();
   }
   
}
