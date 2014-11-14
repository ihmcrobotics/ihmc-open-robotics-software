package us.ihmc.simulationconstructionset.gui;

import java.awt.datatransfer.Transferable;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

public class YoGraphTransferHandler extends TransferHandler
{
   private static final long serialVersionUID = 6982364243920152679L;

   public YoGraphTransferHandler()
   {

   }

   public boolean canImport(TransferSupport transferSupport)
   {
      return false;
   }

   public boolean importData(TransferHandler.TransferSupport transferSupport)
   {
      return false;
   }

   public int getSourceActions(JComponent c)
   {
      if (c instanceof YoGraph)
      {
         return TransferHandler.COPY_OR_MOVE;
      }

      return TransferHandler.NONE;
   }

   public Transferable createTransferable(JComponent c)
   {
      if (c instanceof YoGraph)
      {
         Transferable tip = new VarPanelTransferable();

         return tip;
      }

      return null;
   }

   public void exportDone(JComponent c, Transferable t, int action)
   {

   }
}
