package us.ihmc.simulationconstructionset.gui;

import java.awt.datatransfer.Transferable;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelTransferable;

public class YoGraphTransferHandler extends TransferHandler
{
   private static final long serialVersionUID = 6982364243920152679L;

   public YoGraphTransferHandler()
   {

   }

   @Override
   public boolean canImport(TransferSupport transferSupport)
   {
      return false;
   }

   @Override
   public boolean importData(TransferHandler.TransferSupport transferSupport)
   {
      return false;
   }

   @Override
   public int getSourceActions(JComponent c)
   {
      if (c instanceof YoGraph)
      {
         return TransferHandler.COPY_OR_MOVE;
      }

      return TransferHandler.NONE;
   }

   @Override
   public Transferable createTransferable(JComponent c)
   {
      if (c instanceof YoGraph)
      {
         Transferable tip = new YoVariablePanelTransferable();

         return tip;
      }

      return null;
   }

   @Override
   public void exportDone(JComponent c, Transferable t, int action)
   {

   }
}
