package us.ihmc.simulationconstructionset.gui;

import java.awt.datatransfer.Transferable;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelTransferable;

public class YoEntryBoxTransferHandler extends TransferHandler
{
   private static final long serialVersionUID = 4062923325804002945L;

   public YoEntryBoxTransferHandler()
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
      if (c instanceof YoEntryBox)
      {
         return TransferHandler.COPY_OR_MOVE;
      }

      return TransferHandler.NONE;
   }

   @Override
   public Transferable createTransferable(JComponent c)
   {
      if (c instanceof YoEntryBox)
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
