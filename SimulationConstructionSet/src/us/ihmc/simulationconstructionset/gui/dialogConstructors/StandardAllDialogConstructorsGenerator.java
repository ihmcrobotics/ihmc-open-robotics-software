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
import us.ihmc.simulationconstructionset.movies.ExportMovie;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;

public class StandardAllDialogConstructorsGenerator implements AllDialogConstructorsHolder
{
   private final ExportDataDialogConstructor exportDataDialogConstructor;
   private final ImportDataDialogConstructor importDataDialogConstructor;
   
   private final MediaCaptureDialogGenerator mediaCaptureDialogConstructor;
   private final ExportSnapshotDialogGenerator exportSnapshotDialogConstructor;
   
   private final PlaybackPropertiesDialogGenerator playbackPropertiesDialogConstructor;
   
   private final SaveConfigurationDialogGenerator saveConfigurationDialogConstructor;
   private final LoadConfigurationDialogGenerator loadConfigurationDialogConstructor;
   private final SaveGraphConfigurationDialogGenerator saveGraphConfigurationDialogConstructor;
   private final LoadGraphGroupDialogConstructor loadGraphGroupDialogConstructor;
   private final SaveRobotConfigurationDialogGenerator saveRobotConfigurationDialogConstructor;
   private final ExportSimulationTo3DMaxDialogGenerator exportSimulationTo3DMaxDialogConstructor;
   
   private final PrintGraphsDialogGenerator printGraphsDialogConstructor;
   
   private final DataBufferPropertiesDialogGenerator dataBufferPropertiesDialogConstructor;
   
   private final CameraPropertiesDialogGenerator cameraPropertiesDialogConstructor;
   private final ResizeViewportDialogGenerator resizeViewportDialogConstructor;
   
   private final AboutDialogGenerator aboutDialogConstructor;

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

      ExportMovie exportMovie = new ExportMovie(timeHolder, standardSimulationGUI, dataBufferCommandsExecutor, runCommandsExecutor, 
            guiEnablerAndDisabler, activeCanvas3DHolder, simulationSynchronizer);
      
      this.mediaCaptureDialogConstructor = new MediaCaptureDialogGenerator(exportMovie, guiEnablerAndDisabler, stopCommandExecutor, viewportSelector, standardSimulationGUI, standardGUIActions, activeCanvas3DHolder);
      this.exportSnapshotDialogConstructor = new ExportSnapshotDialogGenerator(sim, sim, robots, myGUI, frame);
      
      this.playbackPropertiesDialogConstructor = new PlaybackPropertiesDialogGenerator(sim, parentContainer, frame);

      this.saveConfigurationDialogConstructor = new SaveConfigurationDialogGenerator(sim, frame, myGUI);
      this.loadConfigurationDialogConstructor = new LoadConfigurationDialogGenerator(sim, frame, myGUI);
      this.saveGraphConfigurationDialogConstructor = new SaveGraphConfigurationDialogGenerator(sim, frame, myGraphArrayPanel);
      this.loadGraphGroupDialogConstructor = new LoadGraphGroupDialogGenerator(sim, myGUI, myGUI, frame, myGraphArrayPanel);

      this.saveRobotConfigurationDialogConstructor = new SaveRobotConfigurationDialogGenerator(sim, frame);
      
      this.exportSimulationTo3DMaxDialogConstructor = new ExportSimulationTo3DMaxDialogGenerator(sim);
      
      this.printGraphsDialogConstructor = new PrintGraphsDialogGenerator(myGraphArrayPanel);
      
      this.dataBufferPropertiesDialogConstructor = new DataBufferPropertiesDialogGenerator(myDataBuffer, parentContainer, frame, myGraphArrayPanel);
      
      this.cameraPropertiesDialogConstructor = new CameraPropertiesDialogGenerator(myGUI, parentContainer, frame);
      this.resizeViewportDialogConstructor = new ResizeViewportDialogGenerator(frame, viewportSelector);

      this.aboutDialogConstructor = new AboutDialogGenerator(frame);
   }
   
   public AboutDialogConstructor getAboutDialogConstructor()
   {
      return aboutDialogConstructor;
   }

   public CameraPropertiesDialogConstructor getCameraPropertiesDialogConstructor()
   {
      return cameraPropertiesDialogConstructor;
   }

   public DataBufferPropertiesDialogConstructor getDataBufferPropertiesDialogConstructor()
   {
      return dataBufferPropertiesDialogConstructor;
   }

   public ExportDataDialogConstructor getExportDataDialogConstructor()
   {
      return exportDataDialogConstructor;
   }

   public ExportSnapshotDialogConstructor getExportSnapshotDialogConstructor()
   {
      return exportSnapshotDialogConstructor;
   }

   public ImportDataDialogConstructor getImportDataDialogConstructor()
   {
      return importDataDialogConstructor;
   }

   public LoadConfigurationDialogConstructor getLoadConfigurationDialogConstructor()
   {
      return loadConfigurationDialogConstructor;
   }

   public MediaCaptureDialogConstructor getMediaCaptureDialogConstructor()
   {
      return mediaCaptureDialogConstructor;
   }

   public PlaybackPropertiesDialogConstructor getPlaybackPropertiesDialogConstructor()
   {
      return playbackPropertiesDialogConstructor;
   }

   public PrintGraphsDialogConstructor getPrintGraphsDialogConstructor()
   {
      return printGraphsDialogConstructor;
   }

   public ResizeViewportDialogConstructor getResizeViewportDialogConstructor()
   {
      return resizeViewportDialogConstructor;
   }

   public SaveConfigurationDialogConstructor getSaveConfigurationDialogConstructor()
   {
      return saveConfigurationDialogConstructor;
   }

   public SaveGraphConfigurationDialogConstructor getSaveGraphConfigurationDialogConstructor()
   {
      return saveGraphConfigurationDialogConstructor;
   }
   
   public LoadGraphGroupDialogConstructor getLoadGraphGroupDialogConstructor()
   {
      return loadGraphGroupDialogConstructor;
   }


   public SaveRobotConfigurationDialogConstructor getSaveRobotConfigurationDialogConstructor()
   {
      return saveRobotConfigurationDialogConstructor;
   }

   public ExportSimulationTo3DMaxDialogConstructor getExportSimulationTo3DMaxDialogConstructor()
   {
      return exportSimulationTo3DMaxDialogConstructor;
   }

}
