package us.ihmc.simulationconstructionset.gui.yoVariableSearch;

import java.awt.datatransfer.Transferable;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

public class YoVariablePanelTransferHandler extends TransferHandler
{
   private static final long serialVersionUID = -3557929937089612422L;
   public static final int YO_VARIABLE_BEING_TRANSFERED = Integer.MAX_VALUE;

   public YoVariablePanelTransferHandler()
   {
      super();
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
      if (c instanceof YoVariablePanel)
      {
         return TransferHandler.COPY;
      }

      return TransferHandler.NONE;
   }

   @Override
   public Transferable createTransferable(JComponent c)
   {
      if (c instanceof YoVariablePanel)
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
