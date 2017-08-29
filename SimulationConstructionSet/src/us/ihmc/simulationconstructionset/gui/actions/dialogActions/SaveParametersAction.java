package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveParametersConstructor;

public class SaveParametersAction extends AbstractAction
{
   private static final long serialVersionUID = -3718283997393463276L;
   private SaveParametersConstructor constructor;
   
   public SaveParametersAction(SaveParametersConstructor constructor)
   {
      super("Save Parameters...");
      this.constructor = constructor;
      
      String iconFilename = "icons/saveParameters.gif";
      int shortKey = KeyEvent.VK_UNDEFINED;
      String longDescription = "Save Parameters To File";
      String shortDescription = "Save Parameters";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);

   }
   
   @Override
   public void actionPerformed(ActionEvent e)
   {
      constructor.constructDialog();
   }
   
   public void closeAndDispose()
   {
      constructor.closeAndDispose();
      constructor = null;
   }

}
