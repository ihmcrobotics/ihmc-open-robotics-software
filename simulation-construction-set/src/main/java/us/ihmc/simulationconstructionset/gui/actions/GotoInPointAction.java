package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;
import us.ihmc.yoVariables.dataBuffer.GotoInPointCommandExecutor;

public class GotoInPointAction extends AbstractAction
{
   private static final long serialVersionUID = -9080927302316708389L;

   private final GotoInPointCommandExecutor executor;

   public GotoInPointAction(GotoInPointCommandExecutor executor)
   {
      super("Goto In Point");
      this.executor = executor;

      String iconFilename = "icons/GotoInPoint.png";
      int shortKey = KeyEvent.VK_I;
      String longDescription = "Goto In Point";
      String shortDescription = "Goto In Point";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      executor.gotoInPoint();
   }
}
