package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.io.File;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveRobotConfigurationDialogConstructor;

public class SaveRobotConfigurationAction extends AbstractAction
{
   private static final long serialVersionUID = 5813345490164040993L;
   private SaveRobotConfigurationDialogConstructor constructor;

   public SaveRobotConfigurationAction(SaveRobotConfigurationDialogConstructor constructor) 
   {
      super("Save Robot Configuration");
      this.constructor = constructor;

      this.putValue(Action.LONG_DESCRIPTION, "Save Robot Configuration");
      this.putValue(Action.SHORT_DESCRIPTION, "save robot config");
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
}
