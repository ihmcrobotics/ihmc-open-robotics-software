package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.io.File;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadGraphGroupDialogConstructor;

public class LoadGraphGroupAction extends AbstractAction
{
   private static final long serialVersionUID = 5813345490164040993L;
   private LoadGraphGroupDialogConstructor constructor;
   
   public LoadGraphGroupAction(LoadGraphGroupDialogConstructor constructor)
   {
      super("Load Graph Group...");
      this.constructor = constructor;
      
      this.putValue(Action.LONG_DESCRIPTION, "Load Graph Group");
      this.putValue(Action.SHORT_DESCRIPTION, "load graphs");
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      constructor.constructDialog();
   }

   public void loadGUIConfigurationFile(File file)
   {
      constructor.loadGraphGroupFile(file);
      
   }
}
