package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.Container;

import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.TimeHolder;
import us.ihmc.simulationconstructionset.commands.DataBufferCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.videos.ExportVideo;

public class StandardAllDialogConstructorsGenerator implements AllDialogConstructorsHolder
{
   private ExportDataDialogConstructor exportDataDialogConstructor;
   private ImportDataDialogConstructor importDataDialogConstructor;
   
   private MediaCaptureDialogGenerator mediaCaptureDialogConstructor;
   private ExportSnapshotDialogGenerator exportSnapshotDialogConstructor;
   
   private PlaybackPropertiesDialogGenerator playbackPropertiesDialogConstructor;
   
   private SaveConfigurationDialogGenerator saveConfigurationDialogConstructor;
   private LoadConfigurationDialogGenerator loadConfigurationDialogConstructor;
   private SaveGraphConfigurationDialogGenerator saveGraphConfigurationDialogConstructor;
   private LoadGraphGroupDialogConstructor loadGraphGroupDialogConstructor;
   private ExportGraphsToFileConstructor exportGraphsToFileConstructor;
   private SaveRobotConfigurationDialogGenerator saveRobotConfigurationDialogConstructor;
   private ExportSimulationTo3DMaxDialogGenerator exportSimulationTo3DMaxDialogConstructor;
   
   private PrintGraphsDialogGenerator printGraphsDialogConstructor;
   
   private DataBufferPropertiesDialogGenerator dataBufferPropertiesDialogConstructor;
   
   private CameraPropertiesDialogGenerator cameraPropertiesDialogConstructor;
   private ResizeViewportDialogGenerator resizeViewportDialogConstructor;
   
   private AboutDialogGenerator aboutDialogConstructor;

   public StandardAllDialogConstructorsGenerator(SimulationConstructionSet sim, Robot[] robots, DataBuffer myDataBuffer,
         StandardSimulationGUI myGUI, 
         VarGroupList varGroupList, GraphArrayPanel myGraphArrayPanel, 
         ViewportSelectorCommandExecutor viewportSelector,
         Container parentContainer, JFrame frame, SimulationSynchronizer simulationSynchronizer, StandardGUIActions standardGUIActions)
   {
      this.importDataDialogConstructor = new ImportDataDialogGenerator(sim, robots, frame);
      this.exportDataDialogConstructor = new ExportDataDialogGenerator(sim, varGroupList, frame);
      
      TimeHolder timeHolder = sim;
      StandardSimulationGUI standardSimulationGUI = myGUI;
      DataBufferCommandsExecutor dataBufferCommandsExecutor = sim;
      RunCommandsExecutor runCommandsExecutor = sim;
      GUIEnablerAndDisabler guiEnablerAndDisabler = sim;
      ActiveCanvas3DHolder activeCanvas3DHolder = myGUI;
      StopCommandExecutor stopCommandExecutor = sim;

      ExportVideo exportVideo = new ExportVideo(timeHolder, standardSimulationGUI, dataBufferCommandsExecutor, runCommandsExecutor, 
            guiEnablerAndDisabler, activeCanvas3DHolder, simulationSynchronizer);
      
      this.mediaCaptureDialogConstructor = new MediaCaptureDialogGenerator(exportVideo, guiEnablerAndDisabler, stopCommandExecutor, viewportSelector, standardSimulationGUI, standardGUIActions, activeCanvas3DHolder);
      this.exportSnapshotDialogConstructor = new ExportSnapshotDialogGenerator(sim, sim, robots, myGUI, frame);
      
      this.playbackPropertiesDialogConstructor = new PlaybackPropertiesDialogGenerator(sim, parentContainer, frame);

      this.saveConfigurationDialogConstructor = new SaveConfigurationDialogGenerator(sim, frame, myGUI);
      this.loadConfigurationDialogConstructor = new LoadConfigurationDialogGenerator(sim, frame, myGUI);
      this.saveGraphConfigurationDialogConstructor = new SaveGraphConfigurationDialogGenerator(sim, frame, myGraphArrayPanel);
      this.loadGraphGroupDialogConstructor = new LoadGraphGroupDialogGenerator(sim, myGUI, myGUI, frame, myGraphArrayPanel);
      
      ExportGraphsToFileConstructor exportGraphsToFileConstructor = new ExportGraphsToFileGenerator(sim, frame, myGraphArrayPanel, myGUI);
      this.exportGraphsToFileConstructor = exportGraphsToFileConstructor;

      this.saveRobotConfigurationDialogConstructor = new SaveRobotConfigurationDialogGenerator(sim, frame);
      
      this.exportSimulationTo3DMaxDialogConstructor = new ExportSimulationTo3DMaxDialogGenerator(sim);
      
      this.printGraphsDialogConstructor = new PrintGraphsDialogGenerator(myGraphArrayPanel);
      
      this.dataBufferPropertiesDialogConstructor = new DataBufferPropertiesDialogGenerator(myDataBuffer, parentContainer, frame, myGraphArrayPanel);
      
      this.cameraPropertiesDialogConstructor = new CameraPropertiesDialogGenerator(myGUI, parentContainer, frame);
      this.resizeViewportDialogConstructor = new ResizeViewportDialogGenerator(frame, viewportSelector);

      this.aboutDialogConstructor = new AboutDialogGenerator(frame);
   }
   
   @Override
   public AboutDialogConstructor getAboutDialogConstructor()
   {
      return aboutDialogConstructor;
   }

   @Override
   public CameraPropertiesDialogConstructor getCameraPropertiesDialogConstructor()
   {
      return cameraPropertiesDialogConstructor;
   }

   @Override
   public DataBufferPropertiesDialogConstructor getDataBufferPropertiesDialogConstructor()
   {
      return dataBufferPropertiesDialogConstructor;
   }

   @Override
   public ExportDataDialogConstructor getExportDataDialogConstructor()
   {
      return exportDataDialogConstructor;
   }

   @Override
   public ExportSnapshotDialogConstructor getExportSnapshotDialogConstructor()
   {
      return exportSnapshotDialogConstructor;
   }

   @Override
   public ImportDataDialogConstructor getImportDataDialogConstructor()
   {
      return importDataDialogConstructor;
   }

   @Override
   public LoadConfigurationDialogConstructor getLoadConfigurationDialogConstructor()
   {
      return loadConfigurationDialogConstructor;
   }

   @Override
   public MediaCaptureDialogConstructor getMediaCaptureDialogConstructor()
   {
      return mediaCaptureDialogConstructor;
   }

   @Override
   public PlaybackPropertiesDialogConstructor getPlaybackPropertiesDialogConstructor()
   {
      return playbackPropertiesDialogConstructor;
   }

   @Override
   public PrintGraphsDialogConstructor getPrintGraphsDialogConstructor()
   {
      return printGraphsDialogConstructor;
   }

   @Override
   public ResizeViewportDialogConstructor getResizeViewportDialogConstructor()
   {
      return resizeViewportDialogConstructor;
   }

   @Override
   public SaveConfigurationDialogConstructor getSaveConfigurationDialogConstructor()
   {
      return saveConfigurationDialogConstructor;
   }

   @Override
   public SaveGraphConfigurationDialogConstructor getSaveGraphConfigurationDialogConstructor()
   {
      return saveGraphConfigurationDialogConstructor;
   }
   
   @Override
   public LoadGraphGroupDialogConstructor getLoadGraphGroupDialogConstructor()
   {
      return loadGraphGroupDialogConstructor;
   }
   
   @Override
   public ExportGraphsToFileConstructor getExportGraphsToFileConstructor()
   {
      return exportGraphsToFileConstructor;
   }


   @Override
   public SaveRobotConfigurationDialogConstructor getSaveRobotConfigurationDialogConstructor()
   {
      return saveRobotConfigurationDialogConstructor;
   }

   @Override
   public ExportSimulationTo3DMaxDialogConstructor getExportSimulationTo3DMaxDialogConstructor()
   {
      return exportSimulationTo3DMaxDialogConstructor;
   }

   private boolean alreadyClosing = false;
   @Override
   public void closeAndDispose()
   {
      if (alreadyClosing) return;
      alreadyClosing = true;
      
      if (exportDataDialogConstructor != null)
      {
         exportDataDialogConstructor.closeAndDispose();
         exportDataDialogConstructor = null;
      }
      
      if (importDataDialogConstructor != null)
      {
         importDataDialogConstructor.closeAndDispose();
         importDataDialogConstructor = null;
      }
      
      if (mediaCaptureDialogConstructor != null)
      {
         mediaCaptureDialogConstructor.closeAndDispose();
         mediaCaptureDialogConstructor = null;
      }
      
      if (exportSnapshotDialogConstructor != null)
      {
         exportSnapshotDialogConstructor.closeAndDispose();
         exportSnapshotDialogConstructor = null;
      }
      
      if (playbackPropertiesDialogConstructor != null)
      {
         playbackPropertiesDialogConstructor.closeAndDispose();
         playbackPropertiesDialogConstructor = null;
      }
      
      if (saveConfigurationDialogConstructor != null)
      {
         saveConfigurationDialogConstructor.closeAndDispose();
         saveConfigurationDialogConstructor = null;
      }
      
      if (loadConfigurationDialogConstructor != null)
      {
         loadConfigurationDialogConstructor.closeAndDispose();
         loadConfigurationDialogConstructor = null;
      }
      
      if (saveGraphConfigurationDialogConstructor != null)
      {
         saveGraphConfigurationDialogConstructor.closeAndDispose();
         saveGraphConfigurationDialogConstructor = null;
      }
      
      if (loadGraphGroupDialogConstructor != null)
      {
         loadGraphGroupDialogConstructor.closeAndDispose();
         loadGraphGroupDialogConstructor = null;
      }
      
      if (saveRobotConfigurationDialogConstructor != null)
      {
         saveRobotConfigurationDialogConstructor.closeAndDispose();
         saveRobotConfigurationDialogConstructor = null;
      }
      
      if (exportSimulationTo3DMaxDialogConstructor != null)
      {
         exportSimulationTo3DMaxDialogConstructor.closeAndDispose();
         exportSimulationTo3DMaxDialogConstructor = null;
      }
      
      if (printGraphsDialogConstructor != null)
      {
         printGraphsDialogConstructor.closeAndDispose();
         printGraphsDialogConstructor = null;
      }
      
      if (dataBufferPropertiesDialogConstructor != null)
      {
         dataBufferPropertiesDialogConstructor.closeAndDispose();
         dataBufferPropertiesDialogConstructor = null;
      }
      
      if (cameraPropertiesDialogConstructor != null)
      {
         cameraPropertiesDialogConstructor.closeAndDispose();
         cameraPropertiesDialogConstructor = null;
      }
      
      if (resizeViewportDialogConstructor != null)
      {
         resizeViewportDialogConstructor.closeAndDispose();
         resizeViewportDialogConstructor = null;
      }
      
      if (aboutDialogConstructor != null)
      {
         aboutDialogConstructor.closeAndDispose();
         aboutDialogConstructor = null;
      }
      
      if (exportGraphsToFileConstructor != null)
      {
         exportGraphsToFileConstructor.closeAndDispose();
         exportGraphsToFileConstructor = null;
      }
      
      
  
   }

}
