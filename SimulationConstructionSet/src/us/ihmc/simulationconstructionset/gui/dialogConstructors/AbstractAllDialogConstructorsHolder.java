package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;

import java.io.File;

public class AbstractAllDialogConstructorsHolder implements AllDialogConstructorsHolder
{
   public AboutDialogConstructor getAboutDialogConstructor()
   {
      return new AboutDialogConstructor()
      {
         public void constructDialog()
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
         public void constructDialog()
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
         @Override
         public void setCurrentDirectory(File directory)
         {
         }

         public void setCurrentDirectory(String directory)
         {
         }

         public void constructDialog()
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

         public void constructDialog()
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


         public void constructDialog()
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


         public void constructDialog()
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


         public void constructDialog()
         {
         }
      };
   }


   public PlaybackPropertiesDialogConstructor getPlaybackPropertiesDialogConstructor()
   {
      return new PlaybackPropertiesDialogConstructor()
      {
         public void constructDialog()
         {
         }
      };
   }


   public PrintGraphsDialogConstructor getPrintGraphsDialogConstructor()
   {
      return new PrintGraphsDialogConstructor()
      {
         public void constructDialog()
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
         public void constructDialog()
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


         public void constructDialog()
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


         public void constructDialog()
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


         public void constructDialog()
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


         public void constructDialog()
         {
         }
      };
   }


   public ExportSimulationTo3DMaxDialogConstructor getExportSimulationTo3DMaxDialogConstructor()
   {
      return new ExportSimulationTo3DMaxDialogConstructor()
      {
         public void constructDialog()
         {
         }
      };
   }


   public void closeAndDispose()
   {      
   }

}
