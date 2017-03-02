package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import javax.swing.JFrame;
import javax.swing.JOptionPane;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;

public class ResizeViewportDialogGenerator implements ResizeViewportDialogConstructor
{
   private JFrame frame;

   public ResizeViewportDialogGenerator(JFrame frame, ViewportSelectorCommandExecutor viewportSelector)
   {
      this.frame = frame;
   }

   @Override
   public void constructDialog()
   {
      String width = JOptionPane.showInputDialog("Enter Viewport Width");
      if (width != null)
      {
         String height = JOptionPane.showInputDialog("Enter Viewport Height");
         if (height != null)
         {
            try
            {
               int intWidth = new Double(width).intValue();
               int intHeight = new Double(height).intValue();
               frame.setSize(intWidth + 16, intHeight + 110);
            }
            catch (NumberFormatException e1)
            {
               // TODO Auto-generated catch block
               e1.printStackTrace();
            }
         }
      }
   }

   public void closeAndDispose()
   {
      this.frame = null;
   }
}

