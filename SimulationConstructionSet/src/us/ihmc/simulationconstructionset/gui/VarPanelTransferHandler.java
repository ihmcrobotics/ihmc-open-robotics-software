package us.ihmc.simulationconstructionset.gui;

import java.awt.datatransfer.Transferable;

import javax.swing.JComponent;
import javax.swing.TransferHandler;

public class VarPanelTransferHandler extends TransferHandler
{
   private static final long serialVersionUID = -3557929937089612422L;
   public static final int YO_VARIABLE_BEING_TRANSFERED = Integer.MAX_VALUE;

   public VarPanelTransferHandler()
   {
      super();
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
      if (c instanceof VarPanel)
      {
         return TransferHandler.COPY;
      }

      return TransferHandler.NONE;
   }

   public Transferable createTransferable(JComponent c)
   {
      if (c instanceof VarPanel)
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
