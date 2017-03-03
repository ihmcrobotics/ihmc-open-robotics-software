package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;

public class AbstractAllDialogConstructorsHolder implements AllDialogConstructorsHolder
{
   @Override
   public AboutDialogConstructor getAboutDialogConstructor()
   {
      return new AboutDialogConstructor()
      {
         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public CameraPropertiesDialogConstructor getCameraPropertiesDialogConstructor()
   {
      return new CameraPropertiesDialogConstructor()
      {
         @Override
         public void constructCameraPropertiesDialog(TrackCheckBox trackCheckBox, DollyCheckBox dollyCheckBox)
         {
         }
      };
   }


   @Override
   public DataBufferPropertiesDialogConstructor getDataBufferPropertiesDialogConstructor()
   {
      return new DataBufferPropertiesDialogConstructor()
      {
         @Override
         public void constructDialog()
         {
         }

         @Override
         public void closeAndDispose()
         {
         }
      };
   }


   @Override
   public ExportDataDialogConstructor getExportDataDialogConstructor()
   {
      return new ExportDataDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(File directory)
         {
         }

         @Override
         public void setCurrentDirectory(String directory)
         {
         }

         @Override
         public void constructDialog()
         {
         }

         @Override
         public void closeAndDispose()
         {
         }
      };
   }


   @Override
   public ExportSnapshotDialogConstructor getExportSnapshotDialogConstructor()
   {
      return new ExportSnapshotDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(File dir)
         {
         }

         @Override
         public void setCurrentDirectory(String directory)
         {
         }

         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public ImportDataDialogConstructor getImportDataDialogConstructor()
   {
      return new ImportDataDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(String directory)
         {
         }


         @Override
         public void setCurrentDirectory(File directory)
         {
         }


         @Override
         public void constructDialog()
         {
         }


         @Override
         public void closeAndDispose()
         {
         }
      };
   }


   @Override
   public LoadConfigurationDialogConstructor getLoadConfigurationDialogConstructor()
   {
      return new LoadConfigurationDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(String directory)
         {
         }


         @Override
         public void setCurrentDirectory(File directory)
         {
         }


         @Override
         public void loadGUIConfigurationFile(File file)
         {
         }


         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public MediaCaptureDialogConstructor getMediaCaptureDialogConstructor()
   {
      return new MediaCaptureDialogConstructor()
      {
         @Override
         public void createVideo(File file)
         {
         }


         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public PlaybackPropertiesDialogConstructor getPlaybackPropertiesDialogConstructor()
   {
      return new PlaybackPropertiesDialogConstructor()
      {
         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public PrintGraphsDialogConstructor getPrintGraphsDialogConstructor()
   {
      return new PrintGraphsDialogConstructor()
      {
         @Override
         public void constructDialog()
         {
         }


         @Override
         public void closeAndDispose()
         {
         }
      };
   }
   
   @Override
   public ExportGraphsToFileConstructor getExportGraphsToFileConstructor()
   {
      return new ExportGraphsToFileConstructor()
      {
         
         @Override
         public void constructDialog()
         {
            
         }
         
         @Override
         public void closeAndDispose()
         {
            
         }
      };
   }


   @Override
   public ResizeViewportDialogConstructor getResizeViewportDialogConstructor()
   {
      return new ResizeViewportDialogConstructor()
      {
         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public SaveConfigurationDialogConstructor getSaveConfigurationDialogConstructor()
   {
      return new SaveConfigurationDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(File directory)
         {
         }


         @Override
         public void setCurrentDirectory(String directory)
         {
         }


         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public SaveGraphConfigurationDialogConstructor getSaveGraphConfigurationDialogConstructor()
   {
      return new SaveGraphConfigurationDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(File directory)
         {
         }


         @Override
         public void setCurrentDirectory(String directory)
         {
         }


         @Override
         public void constructDialog()
         {
         }
      };
   }
   
   @Override
   public LoadGraphGroupDialogConstructor getLoadGraphGroupDialogConstructor()
   {
      return new LoadGraphGroupDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(File directory)
         {
         }


         @Override
         public void setCurrentDirectory(String directory)
         {
         }


         @Override
         public void constructDialog()
         {
         }


         @Override
         public void loadGraphGroupFile(File file)
         {            
         }

         @Override
         public void closeAndDispose()
         {
         }
      };
   }



   @Override
   public SaveRobotConfigurationDialogConstructor getSaveRobotConfigurationDialogConstructor()
   {
      return new SaveRobotConfigurationDialogConstructor()
      {
         @Override
         public void setCurrentDirectory(File directory)
         {
         }


         @Override
         public void setCurrentDirectory(String directory)
         {
         }


         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public ExportSimulationTo3DMaxDialogConstructor getExportSimulationTo3DMaxDialogConstructor()
   {
      return new ExportSimulationTo3DMaxDialogConstructor()
      {
         @Override
         public void constructDialog()
         {
         }
      };
   }


   @Override
   public void closeAndDispose()
   {      
   }

}
