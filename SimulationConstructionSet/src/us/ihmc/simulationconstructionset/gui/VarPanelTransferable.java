package us.ihmc.simulationconstructionset.gui;

import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.io.IOException;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class VarPanelTransferable implements Transferable
{
   private static DataFlavor abstractYoVariableDataFlavor = null;

   public static DataFlavor getAbstractYoVariableDataFlavor()
   {
      if (abstractYoVariableDataFlavor == null)
      {
         try
         {
            abstractYoVariableDataFlavor = new DataFlavor(DataFlavor.javaJVMLocalObjectMimeType + ";class=" + YoVariable.class.getName());
         }
         catch (ClassNotFoundException e)
         {
            e.printStackTrace();
         }
      }

      return abstractYoVariableDataFlavor;
   }

   public Object getTransferData(DataFlavor flavor) throws UnsupportedFlavorException, IOException
   {
      if ((flavor != null) && flavor.equals(getAbstractYoVariableDataFlavor()))
      {
         return this;
      }

      return null;
   }

   public DataFlavor[] getTransferDataFlavors()
   {
      return new DataFlavor[] {getAbstractYoVariableDataFlavor()};
   }

   public boolean isDataFlavorSupported(DataFlavor flavor)
   {
      if ((flavor != null) && flavor.equals(getAbstractYoVariableDataFlavor()))
      {
         return true;
      }

      return false;
   }

}
