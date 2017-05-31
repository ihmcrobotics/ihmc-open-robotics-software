package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSimulationTo3DMaxDialogConstructor;

public class ExportSimulationTo3DMaxAction extends SCSAction
{
   private static final long serialVersionUID = 7582935737436636441L;
   private ExportSimulationTo3DMaxDialogConstructor constructor;
   

   public ExportSimulationTo3DMaxAction(ExportSimulationTo3DMaxDialogConstructor constructor)
   {
      super("Export Simulation To 3DMax",
              "",
              KeyEvent.VK_UNDEFINED,
              "", // TODO
              "" // TODO
      );
      this.constructor = constructor;
   }

   public void doAction()
   {
      constructor.constructDialog();
   }
}
