package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSimulationTo3DMaxDialogConstructor;

import java.awt.event.KeyEvent;

public class ExportSimulationTo3DMaxAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }
}
