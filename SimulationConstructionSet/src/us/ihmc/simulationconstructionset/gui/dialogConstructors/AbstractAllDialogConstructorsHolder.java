package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;

public class AbstractAllDialogConstructorsHolder implements AllDialogConstructorsHolder
{
   public AboutDialogConstructor getAboutDialogConstructor()
   {
      return new AboutDialogConstructor()
      {
         public void constructAboutDialog()
         {
         }
      };
   }


   public CameraPropertiesDialogConstructor getCameraPropertiesDialogConstructor()
   {
      return new CameraPropertiesDialogConstructor()
      {
         public void constructCameraPropertiesDialog(TrackCheckBox trackCheckBox, DollyCheckBox dollyCheckBox)
         {
         }
      };
   }


   public DataBufferPropertiesDialogConstructor getDataBufferPropertiesDialogConstructor()
   {
      return new DataBufferPropertiesDialogConstructor()
      {
         public void constructDataBufferPropertiesDialog()
         {
         }

         public void closeAndDispose()
         {
         }
      };
   }


   public ExportDataDialogConstructor getExportDataDialogConstructor()
   {
      return new ExportDataDialogConstructor()
      {
         public void setCurrentDirectory(String directory)
         {
         }

         public void constructExportDataDialog()
         {
         }

         public void closeAndDispose()
         {
         }
      };
   }


   public ExportSnapshotDialogConstructor getExportSnapshotDialogConstructor()
   {
      return new ExportSnapshotDialogConstructor()
      {
         public void setCurrentDirectory(File dir)
         {
         }

         public void setCurrentDirectory(String directory)
         {
         }

         public void constructExportSnapshotDialog()
         {
         }
      };
   }


   public ImportDataDialogConstructor getImportDataDialogConstructor()
   {
      return new ImportDataDialogConstructor()
      {
         public void setCurrentDirectory(String directory)
         {
         }


         public void setCurrentDirectory(File directory)
         {
         }


         public void constructImportDataDialog()
         {
         }


         public void closeAndDispose()
         {
         }
      };
   }


   public LoadConfigurationDialogConstructor getLoadConfigurationDialogConstructor()
   {
      return new LoadConfigurationDialogConstructor()
      {
         public void setCurrentDirectory(String directory)
         {
         }


         public void setCurrentDirectory(File directory)
         {
         }


         public void loadGUIConfigurationFile(File file)
         {
         }


         public void constructLoadConfigurationDialog()
         {
         }
      };
   }


   public MediaCaptureDialogConstructor getMediaCaptureDialogConstructor()
   {
      return new MediaCaptureDialogConstructor()
      {
         public void createVideo(File file)
         {
         }


         public void constructMediaCaptureDialog()
         {
         }
      };
   }


   public PlaybackPropertiesDialogConstructor getPlaybackPropertiesDialogConstructor()
   {
      return new PlaybackPropertiesDialogConstructor()
      {
         public void constructPlaybackPropertiesDialog()
         {
         }
      };
   }


   public PrintGraphsDialogConstructor getPrintGraphsDialogConstructor()
   {
      return new PrintGraphsDialogConstructor()
      {
         public void constructPrintGraphsDialog()
         {
         }


         public void closeAndDispose()
         {
         }
      };
   }


   public ResizeViewportDialogConstructor getResizeViewportDialogConstructor()
   {
      return new ResizeViewportDialogConstructor()
      {
         public void constructResizeViewportDialog()
         {
         }
      };
   }


   public SaveConfigurationDialogConstructor getSaveConfigurationDialogConstructor()
   {
      return new SaveConfigurationDialogConstructor()
      {
         public void setCurrentDirectory(File directory)
         {
         }


         public void setCurrentDirectory(String directory)
         {
         }


         public void constructSaveConfigurationDialog()
         {
         }
      };
   }


   public SaveGraphConfigurationDialogConstructor getSaveGraphConfigurationDialogConstructor()
   {
      return new SaveGraphConfigurationDialogConstructor()
      {
         public void setCurrentDirectory(File directory)
         {
         }


         public void setCurrentDirectory(String directory)
         {
         }


         public void constructSaveGraphConfigurationDialog()
         {
         }
      };
   }
   
   public LoadGraphGroupDialogConstructor getLoadGraphGroupDialogConstructor()
   {
      return new LoadGraphGroupDialogConstructor()
      {
         public void setCurrentDirectory(File directory)
         {
         }


         public void setCurrentDirectory(String directory)
         {
         }


         public void constructLoadConfigurationDialog()
         {
         }


         public void loadGraphGroupFile(File file)
         {            
         }

         public void closeAndDispose()
         {
         }
      };
   }



   public SaveRobotConfigurationDialogConstructor getSaveRobotConfigurationDialogConstructor()
   {
      return new SaveRobotConfigurationDialogConstructor()
      {
         public void setCurrentDirectory(File directory)
         {
         }


         public void setCurrentDirectory(String directory)
         {
         }


         public void constructRobotConfigurationDialog()
         {
         }
      };
   }


   public ExportSimulationTo3DMaxDialogConstructor getExportSimulationTo3DMaxDialogConstructor()
   {
      return new ExportSimulationTo3DMaxDialogConstructor()
      {
         public void constructExportSimulationTo3DMaxDialog()
         {
         }
      };
   }


   public void closeAndDispose()
   {      
   }

}
