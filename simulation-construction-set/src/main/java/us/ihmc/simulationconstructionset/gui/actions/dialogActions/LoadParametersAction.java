package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadParametersConstructor;

public class LoadParametersAction extends AbstractAction
{
   private static final long serialVersionUID = -3718283997393463276L;
   private LoadParametersConstructor constructor;
   
   public LoadParametersAction(LoadParametersConstructor constructor)
   {
      super("Load Parameters...");
      this.constructor = constructor;
      
      String iconFilename = "icons/loadParameters.gif";
      int shortKey = KeyEvent.VK_UNDEFINED;
      String longDescription = "Load Parameters From File";
      String shortDescription = "Load Parameters";
      
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
