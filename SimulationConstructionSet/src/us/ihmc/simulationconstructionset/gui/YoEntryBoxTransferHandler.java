package us.ihmc.simulationconstructionset.gui;

import java.awt.datatransfer.Transferable;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

public class YoEntryBoxTransferHandler extends TransferHandler
{
   private static final long serialVersionUID = 4062923325804002945L;

   public YoEntryBoxTransferHandler()
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
      if (c instanceof YoEntryBox)
      {
         return TransferHandler.COPY_OR_MOVE;
      }

      return TransferHandler.NONE;
   }

   public Transferable createTransferable(JComponent c)
   {
      if (c instanceof YoEntryBox)
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
