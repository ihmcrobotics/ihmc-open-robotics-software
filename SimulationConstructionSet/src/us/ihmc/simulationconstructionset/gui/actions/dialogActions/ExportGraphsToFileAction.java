package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportGraphsToFileConstructor;

public class ExportGraphsToFileAction extends AbstractAction
{
   /**
    * 
    */
   private static final long serialVersionUID = 7076583053729816960L;
   private ExportGraphsToFileConstructor constructor;

   public ExportGraphsToFileAction(ExportGraphsToFileConstructor constructor)
   {
      super("Export Graphs To File");
      this.constructor = constructor;

      String iconFilename = "icons/exportGraph.png";
      int shortKey = KeyEvent.VK_UNDEFINED;
      String longDescription = "Export Graphs To File";
      String shortDescription = "Export Graphs To File";
      
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
