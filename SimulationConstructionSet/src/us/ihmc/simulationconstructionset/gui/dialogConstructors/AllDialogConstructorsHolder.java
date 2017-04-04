package us.ihmc.simulationconstructionset.gui.dialogConstructors;

public interface AllDialogConstructorsHolder
{
   public abstract AboutDialogConstructor getAboutDialogConstructor();
   public abstract CameraPropertiesDialogConstructor getCameraPropertiesDialogConstructor();
   public abstract DataBufferPropertiesDialogConstructor getDataBufferPropertiesDialogConstructor();
   public abstract ExportDataDialogConstructor getExportDataDialogConstructor();
   public abstract ExportSnapshotDialogConstructor getExportSnapshotDialogConstructor();
   public abstract ImportDataDialogConstructor getImportDataDialogConstructor();
   public abstract LoadConfigurationDialogConstructor getLoadConfigurationDialogConstructor();
   public abstract MediaCaptureDialogConstructor getMediaCaptureDialogConstructor();
   public abstract PlaybackPropertiesDialogConstructor getPlaybackPropertiesDialogConstructor();
   public abstract YoGraphicsPropertiesDialogConstructor getYoGraphicsPropertiesDialogConstructor();
   public abstract PrintGraphsDialogConstructor getPrintGraphsDialogConstructor();
   public abstract ExportGraphsToFileConstructor getExportGraphsToFileConstructor();
   public abstract ResizeViewportDialogConstructor getResizeViewportDialogConstructor();
   public abstract SaveConfigurationDialogConstructor getSaveConfigurationDialogConstructor();
   public abstract SaveGraphConfigurationDialogConstructor getSaveGraphConfigurationDialogConstructor();
   public abstract LoadGraphGroupDialogConstructor getLoadGraphGroupDialogConstructor();
   public abstract SaveRobotConfigurationDialogConstructor getSaveRobotConfigurationDialogConstructor();
   public abstract ExportSimulationTo3DMaxDialogConstructor getExportSimulationTo3DMaxDialogConstructor();
   public abstract void closeAndDispose();
}
