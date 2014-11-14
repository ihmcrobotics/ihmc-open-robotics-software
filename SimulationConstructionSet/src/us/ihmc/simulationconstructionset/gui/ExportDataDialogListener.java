package us.ihmc.simulationconstructionset.gui;



public interface ExportDataDialogListener
{
   public abstract void export(String varGroup, int dataType, int dataFormat, int compression, boolean spreadsheetFormatted);

}
