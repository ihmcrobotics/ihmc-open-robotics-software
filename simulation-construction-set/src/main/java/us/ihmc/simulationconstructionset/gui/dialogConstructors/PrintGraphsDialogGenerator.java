package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.print.PageFormat;
import java.awt.print.PrinterException;
import java.awt.print.PrinterJob;

import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;

public class PrintGraphsDialogGenerator implements PrintGraphsDialogConstructor
{
   private GraphArrayPanel myGraphArrayPanel;

   public PrintGraphsDialogGenerator(GraphArrayPanel graphArrayPanel)
   {
      this.myGraphArrayPanel = graphArrayPanel;
   }

   
   @Override
   public void closeAndDispose()
   {
      myGraphArrayPanel = null;
   }
   
   @Override
   public void constructDialog()
   {
      // System.out.println("Printing..");
      // Paper paper = new Paper();
      // paper.setImageableArea(1*72, 1.5*72, 6.5*72, 8*72);

      // format.setPaper(paper);
      Thread t = new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            PrinterJob printerJob = PrinterJob.getPrinterJob();

            PageFormat format = new PageFormat();
            format = printerJob.pageDialog(format);

            printerJob.setPrintable(myGraphArrayPanel, format);
            printerJob.setJobName("Simulation Construction Set");

            if (printerJob.printDialog())
            {
               try
               {
                  printerJob.print();
               }
               catch (PrinterException pe)
               {
                  System.err.println("printing failed: " + pe.getMessage());
               }
            }
         }
      });
      t.start();
   }

}
