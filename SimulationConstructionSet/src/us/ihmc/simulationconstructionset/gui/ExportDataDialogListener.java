package us.ihmc.simulationconstructionset.gui;

import us.ihmc.simulationconstructionset.gui.dialogs.SCSExportDataFormat;



public interface ExportDataDialogListener
{
   public abstract void export(String varGroup, int dataType, SCSExportDataFormat dataFormat, int compression);

}
