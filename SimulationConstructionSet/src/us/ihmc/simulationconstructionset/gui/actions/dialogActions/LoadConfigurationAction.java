package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadConfigurationDialogConstructor;

public class LoadConfigurationAction extends SCSAction
{
   private static final long serialVersionUID = 5813345490164040993L;
   private LoadConfigurationDialogConstructor constructor;
   
   public LoadConfigurationAction(LoadConfigurationDialogConstructor constructor)
   {
      super("Load Configuration...",
              "",
              KeyEvent.VK_UNDEFINED,
              "Load Config",
              "Load Configuration"
      );

      this.constructor = constructor;
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void doAction()
   {
      constructor.constructDialog();
   }

   public void loadGUIConfigurationFile(File file)
   {
      constructor.loadGUIConfigurationFile(file);
      
   }
}
