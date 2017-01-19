package us.ihmc.simulationconstructionset.gui;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.ArrayList;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JToolBar;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfigurationList;
import us.ihmc.simulationconstructionset.commands.AllCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.SelectGUIConfigFromFileCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.AddCameraKeyAction;
import us.ihmc.simulationconstructionset.gui.actions.AddKeyPointAction;
import us.ihmc.simulationconstructionset.gui.actions.CreateNewGraphWindowAction;
import us.ihmc.simulationconstructionset.gui.actions.CreateNewViewportWindowAction;
import us.ihmc.simulationconstructionset.gui.actions.CropBufferAction;
import us.ihmc.simulationconstructionset.gui.actions.CutBufferAction;
import us.ihmc.simulationconstructionset.gui.actions.GotoInPointAction;
import us.ihmc.simulationconstructionset.gui.actions.GotoOutPointAction;
import us.ihmc.simulationconstructionset.gui.actions.HideShowViewportAction;
import us.ihmc.simulationconstructionset.gui.actions.NextCameraKeyAction;
import us.ihmc.simulationconstructionset.gui.actions.PackBufferAction;
import us.ihmc.simulationconstructionset.gui.actions.PlayAction;
import us.ihmc.simulationconstructionset.gui.actions.PreviousCameraKeyAction;
import us.ihmc.simulationconstructionset.gui.actions.RemoveCameraKeyAction;
import us.ihmc.simulationconstructionset.gui.actions.SelectCameraAction;
import us.ihmc.simulationconstructionset.gui.actions.SelectGUIConfigFromFileAction;
import us.ihmc.simulationconstructionset.gui.actions.SetInPointAction;
import us.ihmc.simulationconstructionset.gui.actions.SetOutPointAction;
import us.ihmc.simulationconstructionset.gui.actions.SimulateAction;
import us.ihmc.simulationconstructionset.gui.actions.StepBackwardAction;
import us.ihmc.simulationconstructionset.gui.actions.StepForwardAction;
import us.ihmc.simulationconstructionset.gui.actions.StopAction;
import us.ihmc.simulationconstructionset.gui.actions.ThinBufferAction;
import us.ihmc.simulationconstructionset.gui.actions.ToggleCameraKeyModeAction;
import us.ihmc.simulationconstructionset.gui.actions.ToggleKeyPointModeAction;
import us.ihmc.simulationconstructionset.gui.actions.ZoomInAction;
import us.ihmc.simulationconstructionset.gui.actions.ZoomOutAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectGraphConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectGraphGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectVarGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectViewportAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AboutAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.CameraPropertiesAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.DataBufferPropertiesAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportDataAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportGraphsToFileAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportSimulationTo3DMaxAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportSnapshotAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ImportDataAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.LoadConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.LoadGraphGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.MediaCaptureAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.OpenH264LicenseAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.PlaybackPropertiesAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.PrintGraphsAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ResizeViewportAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SaveConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SaveGraphConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SaveRobotConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SelectEntryBoxGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SelectExtraPanelAction;
import us.ihmc.simulationconstructionset.gui.config.CameraSelector;
import us.ihmc.simulationconstructionset.gui.config.ConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupList;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupList;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.ViewportConfigurationList;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AllDialogConstructorsHolder;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.CameraPropertiesDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.DataBufferPropertiesDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportDataDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportGraphsToFileConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSimulationTo3DMaxDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ImportDataDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadGraphGroupDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PlaybackPropertiesDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ResizeViewportDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveRobotConfigurationDialogConstructor;

public class StandardGUIActions implements GUIEnablerAndDisabler
{
   private ArrayList<AbstractAction> guiActions = new ArrayList<AbstractAction>();
   private OpenH264LicenseAction openH264LicenseAction;
   private AboutAction aboutAction;
   
   protected JMenu cameraMenu, viewportMenu, viewMenu, extraPanelsMenu, cameraKeysMenu; //TODO: Make private.
   private CameraPropertiesAction cameraPropertiesAction;
   private JMenu configurationMenu;
   private CreateNewGraphWindowAction createNewGraphWindowAction;
   private CreateNewViewportWindowAction createNewViewportWindowAction;
   private CropBufferAction cropBufferAction;
   private PackBufferAction packBufferAction;
   private CutBufferAction cutBufferAction;
   private ThinBufferAction thinBufferAction;
   private DataBufferPropertiesAction dataBufferPropertiesAction;
   private DollyCheckBox dollyCheckBox;
   private JMenu entryBoxGroupsMenu;

   private JMenuItem exitMenuItem;
   private ExportDataAction exportAction;
   private MediaCaptureAction mediaCapture;

   private ExportSnapshotAction exportSnapshotAction;
   private GotoInPointAction goInPointAction;
   private GotoOutPointAction goOutPointAction;

   // protected EditVarGroupsAction editVarGroupsAction;
   private JMenu graphGroupsMenu;
   private HideShowViewportAction hideShowViewportAction;
   private AddCameraKeyAction addCameraKeyAction;
   private RemoveCameraKeyAction removeCameraKeyAction;
   private NextCameraKeyAction nextCameraKeyAction;
   private PreviousCameraKeyAction previousCameraKeyAction;
   private ImportDataAction importDataAction;

   private PlayAction playAction;
   private PlaybackPropertiesAction playbackPropertiesAction;
   private PrintGraphsAction printGraphsAction;
   private ExportGraphsToFileAction exportGraphsToFileAction;
   private ResizeViewportAction resizeViewportAction;
   private JMenu runMenu;

   private SetInPointAction setInPointAction;
   private AddKeyPointAction setKeyAction;
   private SetOutPointAction setOutPointAction;

   private SaveConfigurationAction saveConfigurationAction;
   private SaveGraphConfigurationAction saveGraphConfigurationAction;
   private LoadGraphGroupAction loadGraphConfigurationAction;

   private LoadConfigurationAction loadConfigurationAction;
   private SaveRobotConfigurationAction saveRobotConfigurationAction;
   private ExportSimulationTo3DMaxAction exportSimulationTo3DMaxAction;

   // private LoadRobotConfigurationAction loadRobotConfigurationAction;

   // private SearchForVariableAction searchForVariableAction;
   private SimulateAction simulateAction;
   private StepBackwardAction stepBackwardAction;
   private StepForwardAction stepForwardAction;
   private StopAction stopAction;
   private ToggleKeyPointModeAction toggleKeyPointModeAction;
   private ToggleCameraKeyModeAction toggleCameraKeyModeAction;
   private TrackCheckBox trackCheckBox;
   private JMenu varGroupsMenu;
   private ZoomInAction zoomInAction;
   private ZoomOutAction zoomOutAction;

   public StandardGUIActions()
   {
   }

   protected void createMainWindowActions(AllCommandsExecutor allCommandsExecutor, AllDialogConstructorsHolder allDialogConstructorsHolder,
         ActiveCameraHolder activeCameraHolder)
   {
      // File Menu Items:
      ExportDataDialogConstructor exportDataDialogConstructor = allDialogConstructorsHolder.getExportDataDialogConstructor();
      exportAction = new ExportDataAction(exportDataDialogConstructor);
      guiActions.add(exportAction);

      ImportDataDialogConstructor importDataDialogConstructor = allDialogConstructorsHolder.getImportDataDialogConstructor();
      importDataAction = new ImportDataAction(importDataDialogConstructor);
      guiActions.add(importDataAction);

      MediaCaptureDialogConstructor mediaCaptureDialogConstructor = allDialogConstructorsHolder.getMediaCaptureDialogConstructor();
      mediaCapture = new MediaCaptureAction(mediaCaptureDialogConstructor);
      guiActions.add(mediaCapture);

      ExportSnapshotDialogConstructor exportSnapshotDialogConstructor = allDialogConstructorsHolder.getExportSnapshotDialogConstructor();
      exportSnapshotAction = new ExportSnapshotAction(exportSnapshotDialogConstructor);
      guiActions.add(exportSnapshotAction);

      // Run Menu Items:
      simulateAction = new SimulateAction(allCommandsExecutor);
      guiActions.add(simulateAction);
      playAction = new PlayAction(allCommandsExecutor);
      guiActions.add(playAction);
      stopAction = new StopAction(allCommandsExecutor);
      guiActions.add(stopAction);
      goInPointAction = new GotoInPointAction(allCommandsExecutor);
      guiActions.add(goInPointAction);
      goOutPointAction = new GotoOutPointAction(allCommandsExecutor);
      guiActions.add(goOutPointAction);
      setInPointAction = new SetInPointAction(allCommandsExecutor);
      guiActions.add(setInPointAction);
      setOutPointAction = new SetOutPointAction(allCommandsExecutor);
      guiActions.add(setOutPointAction);
      stepForwardAction = new StepForwardAction(allCommandsExecutor);
      guiActions.add(stepForwardAction);
      stepBackwardAction = new StepBackwardAction(allCommandsExecutor);
      guiActions.add(stepBackwardAction);

      PlaybackPropertiesDialogConstructor playbackPropertiesDialogConstructor = allDialogConstructorsHolder.getPlaybackPropertiesDialogConstructor();
      playbackPropertiesAction = new PlaybackPropertiesAction(playbackPropertiesDialogConstructor);
      guiActions.add(playbackPropertiesAction);

      // Configuration Menu Items:
      SaveConfigurationDialogConstructor saveConfigurationDialogConstructor = allDialogConstructorsHolder.getSaveConfigurationDialogConstructor();
      saveConfigurationAction = new SaveConfigurationAction(saveConfigurationDialogConstructor);
      guiActions.add(saveConfigurationAction);

      LoadConfigurationDialogConstructor loadConfigurationDialogConstructor = allDialogConstructorsHolder.getLoadConfigurationDialogConstructor();
      loadConfigurationAction = new LoadConfigurationAction(loadConfigurationDialogConstructor);
      guiActions.add(loadConfigurationAction);

      SaveGraphConfigurationDialogConstructor saveGraphConfigurationDialogConstructor = allDialogConstructorsHolder
            .getSaveGraphConfigurationDialogConstructor();
      saveGraphConfigurationAction = new SaveGraphConfigurationAction(saveGraphConfigurationDialogConstructor);
      guiActions.add(saveGraphConfigurationAction);

      LoadGraphGroupDialogConstructor loadGraphGroupDialogConstructor = allDialogConstructorsHolder.getLoadGraphGroupDialogConstructor();
      loadGraphConfigurationAction = new LoadGraphGroupAction(loadGraphGroupDialogConstructor);
      guiActions.add(loadGraphConfigurationAction);

      SaveRobotConfigurationDialogConstructor saveRobotConfigurationDialogConstructor = allDialogConstructorsHolder
            .getSaveRobotConfigurationDialogConstructor();
      saveRobotConfigurationAction = new SaveRobotConfigurationAction(saveRobotConfigurationDialogConstructor);
      guiActions.add(saveRobotConfigurationAction);

      ExportSimulationTo3DMaxDialogConstructor exportSimulationTo3DMaxDialogConstructor = allDialogConstructorsHolder
            .getExportSimulationTo3DMaxDialogConstructor();
      exportSimulationTo3DMaxAction = new ExportSimulationTo3DMaxAction(exportSimulationTo3DMaxDialogConstructor);
      guiActions.add(exportSimulationTo3DMaxAction);

      // Graphs Menu Items:
      zoomInAction = new ZoomInAction(allCommandsExecutor);
      guiActions.add(zoomInAction);
      zoomOutAction = new ZoomOutAction(allCommandsExecutor);
      guiActions.add(zoomOutAction);

      PrintGraphsDialogConstructor printGraphsDialogConstructor = allDialogConstructorsHolder.getPrintGraphsDialogConstructor();
      printGraphsAction = new PrintGraphsAction(printGraphsDialogConstructor);
      guiActions.add(printGraphsAction);
      
      exportGraphsToFileAction = new ExportGraphsToFileAction(allDialogConstructorsHolder.getExportGraphsToFileConstructor());
      guiActions.add(exportGraphsToFileAction);
      
      

      // DataBuffer Menu Items:
      cropBufferAction = new CropBufferAction(allCommandsExecutor);
      guiActions.add(cropBufferAction);
      packBufferAction = new PackBufferAction(allCommandsExecutor);
      guiActions.add(packBufferAction);
      cutBufferAction = new CutBufferAction(allCommandsExecutor);
      guiActions.add(cutBufferAction);
      thinBufferAction = new ThinBufferAction(allCommandsExecutor);
      guiActions.add(thinBufferAction);

      DataBufferPropertiesDialogConstructor dataBufferPropertiesDialogConstructor = allDialogConstructorsHolder.getDataBufferPropertiesDialogConstructor();
      dataBufferPropertiesAction = new DataBufferPropertiesAction(dataBufferPropertiesDialogConstructor);
      guiActions.add(dataBufferPropertiesAction);

      toggleKeyPointModeAction = new ToggleKeyPointModeAction(allCommandsExecutor);
      guiActions.add(toggleKeyPointModeAction);
      setKeyAction = new AddKeyPointAction(allCommandsExecutor);
      guiActions.add(setKeyAction);

      // Viewport Menu Items:
      trackCheckBox = new TrackCheckBox(activeCameraHolder);
      trackCheckBox.setRequestFocusEnabled(false);
      dollyCheckBox = new DollyCheckBox(activeCameraHolder);
      dollyCheckBox.setRequestFocusEnabled(false);

      CameraPropertiesDialogConstructor cameraPropertiesDialogConstructor = allDialogConstructorsHolder.getCameraPropertiesDialogConstructor();
      cameraPropertiesAction = new CameraPropertiesAction(cameraPropertiesDialogConstructor, trackCheckBox, dollyCheckBox);
      guiActions.add(cameraPropertiesAction);

      hideShowViewportAction = new HideShowViewportAction(allCommandsExecutor);
      guiActions.add(hideShowViewportAction);

      // Window Menu Items:
      createNewGraphWindowAction = new CreateNewGraphWindowAction(allCommandsExecutor);
      guiActions.add(createNewGraphWindowAction);

      createNewViewportWindowAction = new CreateNewViewportWindowAction(allCommandsExecutor);
      guiActions.add(createNewViewportWindowAction);

      // Help Menu Items:
      AboutDialogConstructor aboutDialogConstructor = allDialogConstructorsHolder.getAboutDialogConstructor();
      aboutAction = new AboutAction(aboutDialogConstructor);
      openH264LicenseAction = new OpenH264LicenseAction();
      guiActions.add(aboutAction);
      guiActions.add(openH264LicenseAction);

      // Future possibilities:
      // SearchForVariableAction, ToggleKeyPointModeAction, EditVarGroupsAction
   }

   protected void createViewportWindowActions(StandardGUIActions actions, ExportSnapshotDialogConstructor exportSnapshotDialogConstructor,
         MediaCaptureDialogConstructor mediaCaptureDialogConstructor, CameraPropertiesDialogConstructor cameraPropertiesDialogConstructor,
         ResizeViewportDialogConstructor resizeViewportDialogConstructor, ActiveCameraHolder activeCameraHolder,
         ViewportSelectorCommandExecutor viewportSelector)
   {
      // File Menu Items:
      // Use same ExportData, ImportData. New ExportSnapshot and MediaCapture
      exportAction = actions.exportAction;
      guiActions.add(exportAction);
      importDataAction = actions.importDataAction;
      guiActions.add(importDataAction);

      exportSnapshotAction = new ExportSnapshotAction(exportSnapshotDialogConstructor);
      guiActions.add(exportSnapshotAction);

      mediaCapture = new MediaCaptureAction(mediaCaptureDialogConstructor);
      guiActions.add(mediaCapture);

      // Run Menu Items:
      // Use all the same.
      simulateAction = actions.simulateAction;
      guiActions.add(simulateAction);
      playAction = actions.playAction;
      guiActions.add(playAction);
      stopAction = actions.stopAction;
      guiActions.add(stopAction);
      goInPointAction = actions.goInPointAction;
      guiActions.add(goInPointAction);
      goOutPointAction = actions.goOutPointAction;
      guiActions.add(goOutPointAction);
      setInPointAction = actions.setInPointAction;
      guiActions.add(setInPointAction);
      setOutPointAction = actions.setOutPointAction;
      guiActions.add(setOutPointAction);
      stepForwardAction = actions.stepForwardAction;
      guiActions.add(stepForwardAction);
      stepBackwardAction = actions.stepBackwardAction;
      guiActions.add(stepBackwardAction);
      playbackPropertiesAction = actions.playbackPropertiesAction;
      guiActions.add(playbackPropertiesAction);

      // Configuration Menu Items: None.
      // Graphs Menu Items: None.
      // DataBuffer Menu Items: None.

      // Viewport Menu Items: New: Check Boxes, CameraProperties, HideShowViewport, ResizeViewport
      trackCheckBox = new TrackCheckBox(activeCameraHolder);
      trackCheckBox.setRequestFocusEnabled(false);
      dollyCheckBox = new DollyCheckBox(activeCameraHolder);
      dollyCheckBox.setRequestFocusEnabled(false);

      cameraPropertiesAction = new CameraPropertiesAction(cameraPropertiesDialogConstructor, trackCheckBox, dollyCheckBox);
      guiActions.add(cameraPropertiesAction);

      hideShowViewportAction = new HideShowViewportAction(viewportSelector);
      guiActions.add(hideShowViewportAction);

      resizeViewportAction = new ResizeViewportAction(resizeViewportDialogConstructor);
      guiActions.add(resizeViewportAction);

      // Window Menu Items: 
      createNewGraphWindowAction = actions.createNewGraphWindowAction;
      guiActions.add(createNewGraphWindowAction);
      createNewViewportWindowAction = actions.createNewViewportWindowAction;
      guiActions.add(createNewViewportWindowAction);

      // Help Menu Items: None
   }

   public void createVideoExportActions(StandardGUIActions actions, ActiveCameraHolder activeCameraHolder)
   {

      // Run Menu Items:
      // Use all the same.
      simulateAction = actions.simulateAction;
      guiActions.add(simulateAction);
      playAction = actions.playAction;
      guiActions.add(playAction);
      stopAction = actions.stopAction;
      guiActions.add(stopAction);
      goInPointAction = actions.goInPointAction;
      guiActions.add(goInPointAction);
      goOutPointAction = actions.goOutPointAction;
      guiActions.add(goOutPointAction);
      setInPointAction = actions.setInPointAction;
      guiActions.add(setInPointAction);
      setOutPointAction = actions.setOutPointAction;
      guiActions.add(setOutPointAction);
      stepForwardAction = actions.stepForwardAction;
      guiActions.add(stepForwardAction);
      stepBackwardAction = actions.stepBackwardAction;
      guiActions.add(stepBackwardAction);
      playbackPropertiesAction = actions.playbackPropertiesAction;
      guiActions.add(playbackPropertiesAction);

      // Configuration Menu Items: None.
      // Graphs Menu Items: None.
      // DataBuffer Menu Items: None.

      // Viewport Menu Items: New: Check Boxes, CameraProperties, HideShowViewport, ResizeViewport
      trackCheckBox = new TrackCheckBox(activeCameraHolder);
      trackCheckBox.setRequestFocusEnabled(false);
      dollyCheckBox = new DollyCheckBox(activeCameraHolder);
      dollyCheckBox.setRequestFocusEnabled(false);

      toggleKeyPointModeAction = actions.toggleKeyPointModeAction; //new ToggleKeyPointModeAction(allCommandsExecutor);
      guiActions.add(toggleKeyPointModeAction);
      setKeyAction = actions.setKeyAction; //new AddKeyPointAction(allCommandsExecutor);
      guiActions.add(setKeyAction);

   }

   protected void createGraphWindowActions(StandardGUIActions actions, ZoomGraphCommandExecutor zoomGraphCommandExecutor,
         SaveGraphConfigurationDialogConstructor saveGraphConfigurationDialogConstructor, LoadGraphGroupDialogConstructor loadGraphGroupDialogConstructor,
         PrintGraphsDialogConstructor printGraphsDialogConstructor, ExportGraphsToFileConstructor exportGraphsToFileConstructor)
   {
      // File Menu Items:
      exportAction = actions.exportAction;
      guiActions.add(exportAction);
      importDataAction = actions.importDataAction;
      guiActions.add(importDataAction);

      // Run Menu Items:
      simulateAction = actions.simulateAction;
      guiActions.add(simulateAction);
      playAction = actions.playAction;
      guiActions.add(playAction);
      stopAction = actions.stopAction;
      guiActions.add(stopAction);
      goInPointAction = actions.goInPointAction;
      guiActions.add(goInPointAction);
      goOutPointAction = actions.goOutPointAction;
      guiActions.add(goOutPointAction);
      setInPointAction = actions.setInPointAction;
      guiActions.add(setInPointAction);
      setOutPointAction = actions.setOutPointAction;
      guiActions.add(setOutPointAction);
      stepForwardAction = actions.stepForwardAction;
      guiActions.add(stepForwardAction);
      stepBackwardAction = actions.stepBackwardAction;
      guiActions.add(stepBackwardAction);
      playbackPropertiesAction = actions.playbackPropertiesAction;
      guiActions.add(playbackPropertiesAction);
      toggleKeyPointModeAction = actions.toggleKeyPointModeAction; //new ToggleKeyPointModeAction(allCommandsExecutor);
      guiActions.add(toggleKeyPointModeAction);
      setKeyAction = actions.setKeyAction; //new AddKeyPointAction(allCommandsExecutor);
      guiActions.add(setKeyAction);

      // Configuration Menu Items:
      saveGraphConfigurationAction = new SaveGraphConfigurationAction(saveGraphConfigurationDialogConstructor);
      guiActions.add(saveGraphConfigurationAction);

      loadGraphConfigurationAction = new LoadGraphGroupAction(loadGraphGroupDialogConstructor);
      guiActions.add(loadGraphConfigurationAction);

      // Graphs Menu Items:
      zoomInAction = new ZoomInAction(zoomGraphCommandExecutor);
      guiActions.add(zoomInAction);
      zoomOutAction = new ZoomOutAction(zoomGraphCommandExecutor);
      guiActions.add(zoomOutAction);

      printGraphsAction = new PrintGraphsAction(printGraphsDialogConstructor);
      guiActions.add(printGraphsAction);
      
      exportGraphsToFileAction = new ExportGraphsToFileAction(exportGraphsToFileConstructor);
      guiActions.add(exportGraphsToFileAction);

      // DataBuffer Menu Items:
      cropBufferAction = actions.cropBufferAction;
      guiActions.add(cropBufferAction);
      packBufferAction = actions.packBufferAction;
      guiActions.add(packBufferAction);
      cutBufferAction = actions.cutBufferAction;
      guiActions.add(cutBufferAction);
      thinBufferAction = actions.thinBufferAction;
      guiActions.add(thinBufferAction);
      dataBufferPropertiesAction = actions.dataBufferPropertiesAction;
      guiActions.add(dataBufferPropertiesAction);

      // Viewport Menu Items: None

      // Window Menu Items:
      createNewGraphWindowAction = actions.createNewGraphWindowAction;
      guiActions.add(createNewGraphWindowAction);
      createNewViewportWindowAction = actions.createNewViewportWindowAction;
      guiActions.add(createNewViewportWindowAction);
   }

   protected JPanel createWindowButtons(Action[][] allActions, JToolBar[] toolBars, boolean trackAndDolly)
   {
      for (int j = 0; j < allActions.length; j++)
      {
         Action[] actions = allActions[j];

         toolBars[j] = new JToolBar();
         toolBars[j].setBorderPainted(true); // false);
         toolBars[j].setFloatable(false);

         for (Action action : actions)
         {
            if (action.getValue(Action.SMALL_ICON) != null)
            {
               JButton button = toolBars[j].add(action);
               String name = (String) action.getValue(Action.NAME);
               button.setName(name);
               button.setToolTipText(name);
            }
         }
      }

      JPanel buttonPanel = new JPanel(new FlowLayout());

      for (JToolBar toolBar : toolBars)
      {
         buttonPanel.add(toolBar);
      }

      if (trackAndDolly)
      {
         buttonPanel.add(trackCheckBox);
         buttonPanel.add(dollyCheckBox);
      }

      return buttonPanel;
   }

   protected JPanel createMainWindowButtons()
   {
      Action[] fileActions = new Action[] { exportAction, importDataAction, mediaCapture, exportSnapshotAction }; // , searchForVariableAction};
      Action[] runActions = new Action[] { simulateAction, playAction, stopAction };
      Action[] stepActions = new Action[] { setInPointAction, goInPointAction, stepBackwardAction, stepForwardAction, goOutPointAction, setOutPointAction };
      Action[] keyActions = new Action[] { setKeyAction, toggleKeyPointModeAction };
      Action[] playbackPropertiesActions = new Action[] {}; // playbackPropertiesAction};
      Action[] graphsActions = new Action[] { zoomInAction, zoomOutAction, printGraphsAction, exportGraphsToFileAction };
      Action[] cameraActions = new Action[] {}; // cameraPropertiesAction;
      Action[] windowActions = new Action[] {};
      Action[] helpActions = new Action[] {};
      Action[][] allActions = new Action[][] { fileActions, runActions, stepActions, keyActions, playbackPropertiesActions, graphsActions, cameraActions,
            windowActions, helpActions };
      JToolBar[] toolBars = new JToolBar[allActions.length];

      // System.out.println(allActions.length);
      return createWindowButtons(allActions, toolBars, true);
   }

   protected JMenuBar createMainWindowMenus(final ExitActionListenerNotifier exitActionListenerNotifier)
   {
      // Menu:
      // Need to do the following since menus are lightweight and the Canvas3D is heavyweight:
      JPopupMenu.setDefaultLightWeightPopupEnabled(false);

      JMenuBar menuBar = new JMenuBar();

      // File Menu:
      JMenu fileMenu = new JMenu("File");
      fileMenu.setName("File");
      fileMenu.setMnemonic('f');
      menuBar.add(fileMenu);
      exitMenuItem = new JMenuItem("Exit");
      exitMenuItem.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            exitActionListenerNotifier.notifyExitActionListeners();
         }
      });

      // fileMenu.add(exportDataAction);
      fileMenu.add(exportAction);
      fileMenu.add(importDataAction);

      fileMenu.addSeparator();

      fileMenu.add(mediaCapture);
      fileMenu.add(exportSnapshotAction);
      fileMenu.addSeparator();

      // fileMenu.add(searchForVariableAction);
      // fileMenu.addSeparator();
      fileMenu.add(exitMenuItem);

      // Run Menu:
      runMenu = new JMenu("Run");
      runMenu.setName("Run");
      runMenu.setMnemonic('r');
      menuBar.add(runMenu);
      runMenu.add(simulateAction);
      runMenu.add(playAction);
      runMenu.add(stopAction);
      runMenu.addSeparator();
      runMenu.add(goInPointAction);
      runMenu.add(goOutPointAction);
      runMenu.add(setInPointAction);
      runMenu.add(setOutPointAction);
      runMenu.addSeparator();
      runMenu.add(stepForwardAction);
      runMenu.add(stepBackwardAction);
      runMenu.addSeparator();
      runMenu.add(playbackPropertiesAction);

      // Configuration Menu:
      configurationMenu = new JMenu("Configuration");
      configurationMenu.setName("Configuration");
      configurationMenu.setMnemonic('o');
      menuBar.add(configurationMenu);

      // VarGroups Menu:
      varGroupsMenu = new JMenu("VarGroups");
      varGroupsMenu.setName("VarGroups");
      varGroupsMenu.setName("VarGroups");
      varGroupsMenu.setMnemonic('v');
      configurationMenu.add(varGroupsMenu);

      // varGroupsMenu.add(editVarGroupsAction);
      // GraphGroups Menu:
      graphGroupsMenu = new JMenu("GraphGroups");
      graphGroupsMenu.setName("GraphGroups");
      graphGroupsMenu.setMnemonic('g');
      configurationMenu.add(graphGroupsMenu);

      // EntryBoxGroups Menu:
      entryBoxGroupsMenu = new JMenu("EntryBoxGroups");
      entryBoxGroupsMenu.setName("EntryBoxGroups");
      graphGroupsMenu.setMnemonic('e');
      configurationMenu.add(entryBoxGroupsMenu);
      configurationMenu.addSeparator();
      configurationMenu.add(saveConfigurationAction);
      configurationMenu.add(loadConfigurationAction);
      configurationMenu.add(saveRobotConfigurationAction);
      configurationMenu.add(exportSimulationTo3DMaxAction);

      // configurationMenu.add(loadRobotConfigurationAction);

      // Graphs Menu:
      JMenu graphsMenu = new JMenu("Graphs");
      graphsMenu.setName("Graphs");
      graphsMenu.setMnemonic('g');
      menuBar.add(graphsMenu);
      graphsMenu.add(zoomInAction);
      graphsMenu.add(zoomOutAction);
      graphsMenu.addSeparator();
      graphsMenu.add(printGraphsAction);
      graphsMenu.add(exportGraphsToFileAction);

      // Data Buffer Menu:
      JMenu dataBufferMenu = new JMenu("Data Buffer");
      dataBufferMenu.setName("Data Buffer");
      dataBufferMenu.setMnemonic('d');
      menuBar.add(dataBufferMenu);
      dataBufferMenu.add(cropBufferAction);
      dataBufferMenu.add(packBufferAction);
      dataBufferMenu.add(cutBufferAction);
      dataBufferMenu.add(thinBufferAction);
      dataBufferMenu.add(dataBufferPropertiesAction);

      // Viewports and Camera Menu:
      viewportMenu = new JMenu("Viewport");
      viewportMenu.setName("Viewport");
      viewportMenu.setMnemonic('v');
      viewportMenu.add(cameraPropertiesAction);
      viewportMenu.add(hideShowViewportAction);
      cameraMenu = new JMenu("Camera");
      cameraMenu.setName("Camera");
      cameraMenu.setMnemonic('c');
      viewportMenu.add(cameraMenu);
      viewMenu = new JMenu("Views");
      viewMenu.setName("Views");
      viewportMenu.add(viewMenu);
      extraPanelsMenu = new JMenu("Extra Panels");
      extraPanelsMenu.setName("Extra Panels");
      viewportMenu.add(extraPanelsMenu);
      cameraKeysMenu = new JMenu("Camera Keys");
      cameraKeysMenu.setName("Camera Keys");

      // viewportMenu.add(cameraKeys);
      // cameraKeys.add(addCameraKeyAction);

      menuBar.add(viewportMenu);

      // Window Menu:
      JMenu windowMenu = new JMenu("Window");
      windowMenu.setName("Window");
      windowMenu.setMnemonic('w');

      windowMenu.add(createNewGraphWindowAction);
      windowMenu.add(createNewViewportWindowAction);
      menuBar.add(windowMenu);

      // Help Menu:
      JMenu helpMenu = new JMenu("Help");
      helpMenu.setName("Help");
      helpMenu.setMnemonic('h');
      menuBar.add(helpMenu);
      helpMenu.add(openH264LicenseAction);
      helpMenu.add(aboutAction);

      return menuBar;
   }

   public JPanel createVideoExportPanelButtons()
   {
      Action[] fileActions = new Action[] {};
      Action[] runActions = new Action[] { playAction, stopAction };
      Action[] stepActions = new Action[] { goInPointAction, stepBackwardAction, stepForwardAction, goOutPointAction };
      Action[] keyActions = new Action[] { setKeyAction, toggleKeyPointModeAction };

      Action[] playbackPropertiesActions = new Action[] {}; // playbackPropertiesAction};
      Action[] graphsActions = new Action[] {}; // zoomInAction, zoomOutAction, printGraphsAction};
      Action[] cameraActions = new Action[] {}; // cameraPropertiesAction;
      Action[] windowActions = new Action[] {};
      Action[] helpActions = new Action[] {};
      Action[][] allActions = new Action[][] { fileActions, runActions, stepActions, keyActions, playbackPropertiesActions, graphsActions, cameraActions,
            windowActions, helpActions };
      JToolBar[] toolBars = new JToolBar[allActions.length];

      // System.out.println(allActions.length);
      return createWindowButtons(allActions, toolBars, true);
   }

   protected JPanel createViewportWindowButtons()
   {
      Action[] fileActions = new Action[] { exportAction, importDataAction, mediaCapture, exportSnapshotAction };
      Action[] runActions = new Action[] { simulateAction, playAction, stopAction };
      Action[] stepActions = new Action[] { setInPointAction, goInPointAction, stepBackwardAction, stepForwardAction, goOutPointAction, setOutPointAction };
      Action[] playbackPropertiesActions = new Action[] {}; // playbackPropertiesAction};
      Action[] graphsActions = new Action[] {}; // zoomInAction, zoomOutAction, printGraphsAction};
      Action[] cameraActions = new Action[] {}; // cameraPropertiesAction;
      Action[] windowActions = new Action[] {};
      Action[] helpActions = new Action[] {};
      Action[][] allActions = new Action[][] { fileActions, runActions, stepActions, playbackPropertiesActions, graphsActions, cameraActions, windowActions,
            helpActions };
      JToolBar[] toolBars = new JToolBar[allActions.length];

      // System.out.println(allActions.length);
      return createWindowButtons(allActions, toolBars, true);
   }

   protected JMenuBar createViewportWindowMenus()
   {
      // Need to do the following since menus are lightweight and the Canvas3D is heavyweight:
      JPopupMenu.setDefaultLightWeightPopupEnabled(false);

      JMenuBar menuBar = new JMenuBar();

      // File Menu:
      JMenu fileMenu = new JMenu("File");
      fileMenu.setMnemonic('f');
      menuBar.add(fileMenu);

      fileMenu.add(exportAction);
      fileMenu.add(importDataAction);
      fileMenu.addSeparator();
      fileMenu.add(mediaCapture);
      fileMenu.add(exportSnapshotAction);

      // Run Menu:
      runMenu = new JMenu("Run");
      runMenu.setMnemonic('r');
      menuBar.add(runMenu);

      runMenu.add(simulateAction);
      runMenu.add(playAction);
      runMenu.add(stopAction);
      runMenu.addSeparator();
      runMenu.add(goInPointAction);
      runMenu.add(goOutPointAction);
      runMenu.add(setInPointAction);
      runMenu.add(setOutPointAction);
      runMenu.addSeparator();
      runMenu.add(stepForwardAction);
      runMenu.add(stepBackwardAction);
      runMenu.addSeparator();
      runMenu.add(playbackPropertiesAction);

      // Configuration Menu:

      // Viewports Menu:
      viewportMenu = new JMenu("Viewport");
      viewportMenu.setMnemonic('v');
      viewportMenu.add(cameraPropertiesAction);
      viewportMenu.add(hideShowViewportAction);
      viewportMenu.add(resizeViewportAction);

      cameraMenu = new JMenu("Camera");
      cameraMenu.setMnemonic('c');
      viewportMenu.add(cameraMenu);
      menuBar.add(viewportMenu);

      // Window Menu:
      JMenu windowMenu = new JMenu("Window");

      windowMenu.setMnemonic('w');

      windowMenu.add(createNewGraphWindowAction);
      windowMenu.add(createNewViewportWindowAction);
      menuBar.add(windowMenu);

      return menuBar;
   }

   protected JPanel createGraphWindowButtons()
   {
      Action[] fileActions = new Action[] { exportAction, importDataAction }; // , exportVideoAction, exportSnapshotAction};
      Action[] runActions = new Action[] { simulateAction, playAction, stopAction };
      Action[] stepActions = new Action[] { setInPointAction, goInPointAction, stepBackwardAction, stepForwardAction, goOutPointAction, setOutPointAction };
      Action[] keyActions = new Action[] { setKeyAction, toggleKeyPointModeAction };
      Action[] playbackPropertiesActions = new Action[] {}; // playbackPropertiesAction};
      Action[] graphsActions = new Action[] { zoomInAction, zoomOutAction, printGraphsAction, exportGraphsToFileAction };
      Action[] cameraActions = new Action[] {}; // cameraPropertiesAction;
      Action[] windowActions = new Action[] {};
      Action[] helpActions = new Action[] {};
      Action[][] allActions = new Action[][] { fileActions, runActions, stepActions, keyActions, playbackPropertiesActions, graphsActions, cameraActions,
            windowActions, helpActions };
      JToolBar[] toolBars = new JToolBar[allActions.length];

      return createWindowButtons(allActions, toolBars, false);
   }

   protected JMenuBar createGraphWindowMenus()
   {
      // Menu:
      // Need to do the following since menus are lightweight and the Canvas3D is heavyweight:
      JPopupMenu.setDefaultLightWeightPopupEnabled(false);

      JMenuBar menuBar = new JMenuBar();

      // File Menu:
      JMenu fileMenu = new JMenu("File");

      fileMenu.setMnemonic('f');
      menuBar.add(fileMenu);

      fileMenu.add(exportAction);
      fileMenu.add(importDataAction);

      // Run Menu:
      runMenu = new JMenu("Run");
      runMenu.setMnemonic('r');
      menuBar.add(runMenu);
      runMenu.add(simulateAction);
      runMenu.add(playAction);
      runMenu.add(stopAction);
      runMenu.addSeparator();
      runMenu.add(goInPointAction);
      runMenu.add(goOutPointAction);
      runMenu.add(setInPointAction);
      runMenu.add(setOutPointAction);
      runMenu.addSeparator();
      runMenu.add(stepForwardAction);
      runMenu.add(stepBackwardAction);
      runMenu.addSeparator();
      runMenu.add(playbackPropertiesAction);

      // Configuration Menu:
      configurationMenu = new JMenu("Configuration");
      configurationMenu.setMnemonic('o');
      menuBar.add(configurationMenu);

      // GraphGroups Menu:
      graphGroupsMenu = new JMenu("GraphGroups");
      graphGroupsMenu.setMnemonic('g');
      configurationMenu.add(graphGroupsMenu);

      // Graphs Menu:
      JMenu graphsMenu = new JMenu("Graphs");

      graphsMenu.setMnemonic('g');
      menuBar.add(graphsMenu);
      graphsMenu.add(zoomInAction);
      graphsMenu.add(zoomOutAction);
      graphsMenu.addSeparator();
      graphsMenu.add(printGraphsAction);
      graphsMenu.add(exportGraphsToFileAction);

      // Data Buffer Menu:
      JMenu dataBufferMenu = new JMenu("Data Buffer");

      dataBufferMenu.setMnemonic('d');
      menuBar.add(dataBufferMenu);
      dataBufferMenu.add(cropBufferAction);
      dataBufferMenu.add(packBufferAction);
      dataBufferMenu.add(cutBufferAction);
      dataBufferMenu.add(thinBufferAction);
      dataBufferMenu.add(dataBufferPropertiesAction);

      // Window Menu:
      JMenu windowMenu = new JMenu("Window");

      windowMenu.setMnemonic('w');
      menuBar.add(windowMenu);
      windowMenu.add(createNewGraphWindowAction);
      windowMenu.add(createNewViewportWindowAction);

      return menuBar;
   }

   protected void setupConfigurationMenu(final ConfigurationList configurationList, final SelectGraphConfigurationCommandExecutor configurationSelector,
         final SelectGUIConfigFromFileCommandExecutor selectGUIConfigFromFileCommandExecutor)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            setupConfigurationMenuThreadUnsafe(configurationList, configurationSelector, selectGUIConfigFromFileCommandExecutor);
         }
      });
   }

   private void setupConfigurationMenuThreadUnsafe(ConfigurationList configurationList, SelectGraphConfigurationCommandExecutor configurationSelector,
         SelectGUIConfigFromFileCommandExecutor selectGUIConfigFromFileCommandExecutor)
   {
      configurationMenu.removeAll();

      if (saveConfigurationAction != null)
      {
         configurationMenu.add(saveConfigurationAction);
      }

      if (loadConfigurationAction != null)
      {
         configurationMenu.add(loadConfigurationAction);
      }

      if (saveGraphConfigurationAction != null)
      {
         configurationMenu.add(saveGraphConfigurationAction);
      }
      if (loadGraphConfigurationAction != null)
      {
         configurationMenu.add(loadGraphConfigurationAction);
      }

      if (saveRobotConfigurationAction != null)
      {
         configurationMenu.add(saveRobotConfigurationAction);
      }

      if (exportSimulationTo3DMaxAction != null)
      {
         configurationMenu.add(exportSimulationTo3DMaxAction);
      }

      // if (loadRobotConfigurationAction != null)
      // configurationMenu.add(loadRobotConfigurationAction);

      configurationMenu.addSeparator();

      String[] names = configurationList.getConfigurationNames();

      for (int i = 0; i < names.length; i++)
      {
         configurationMenu.add(new SelectGraphConfigurationAction(configurationSelector, names[i]));
      }

      configurationMenu.addSeparator();

      if (varGroupsMenu != null)
      {
         configurationMenu.add(varGroupsMenu);
      }

      if (graphGroupsMenu != null)
      {
         configurationMenu.add(graphGroupsMenu);
      }

      if (entryBoxGroupsMenu != null)
      {
         configurationMenu.add(entryBoxGroupsMenu);
      }

      configurationMenu.addSeparator();

      String path = GUIConfigurationSaveAndLoad.getConfigurationDirectoryPath();
      String[] potentialConfigurationFilenames = GUIConfigurationSaveAndLoad.getPotentialConfigurationFilenames();

      for (String child : potentialConfigurationFilenames)
      {
         if (!child.endsWith(".guiConf"))
            continue;

         int index = child.indexOf(".");
         String name = child.substring(0, index);
         configurationMenu.add(new SelectGUIConfigFromFileAction(path + "/" + child, name, selectGUIConfigFromFileCommandExecutor));
      }
   }

   protected void setupGraphGroupsMenu(final GraphGroupList graphGroupList, final GraphGroupSelector graphGroupSelector)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            graphGroupsMenu.removeAll();

            String[] names = graphGroupList.getGraphGroupNames();
            java.util.Arrays.sort(names);

            for (String name : names)
            {
               graphGroupsMenu.add(new SelectGraphGroupAction(graphGroupSelector, name));
            }
         }
      });

   }

   protected void setupEntryBoxGroupMenu(final EntryBoxGroupList entryBoxGroupList, final EntryBoxGroupSelector entryBoxGroupSelector)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            entryBoxGroupsMenu.removeAll();

            String[] names = entryBoxGroupList.getEntryBoxGroupNames();
            java.util.Arrays.sort(names);

            for (String name : names)
            {
               entryBoxGroupsMenu.add(new SelectEntryBoxGroupAction(entryBoxGroupSelector, name));
            }
         }
      });

   }

   protected void setupViewportMenu(AllCommandsExecutor allCommandsExecutor, ViewportConfigurationList viewportConfigurationList,
         ViewportSelectorCommandExecutor viewportSelector) // ++++++
   {
      if (viewportMenu == null)
      {
         viewportMenu = new JMenu();
      }
      else
      {
         viewportMenu.removeAll();
      }

      if (viewMenu != null)
      {
         viewMenu.removeAll();
      }
      else
      {
         viewMenu = new JMenu("Views");
      }

      String[] names = viewportConfigurationList.getViewportConfigurationNames();

      for (int i = 0; i < names.length; i++)
      {
         viewMenu.add(new SelectViewportAction(viewportSelector, names[i]));
      }

      if (cameraMenu == null)
      {
         cameraMenu = new JMenu();
      }

      if (extraPanelsMenu == null)
      {
         // ExtraPanels = new JMenu("Extra Panels");

      }
      else
      {
         // ExtraPanels.removeAll();
      }

      if (cameraKeysMenu == null)
      {
         cameraKeysMenu = new JMenu("Camera Keys");
      }

      if (addCameraKeyAction == null)
      {
         addCameraKeyAction = new AddCameraKeyAction(allCommandsExecutor);
         cameraKeysMenu.add(addCameraKeyAction);
      }

      if (removeCameraKeyAction == null)
      {
         removeCameraKeyAction = new RemoveCameraKeyAction(allCommandsExecutor);
         cameraKeysMenu.add(removeCameraKeyAction);
      }

      if (nextCameraKeyAction == null)
      {
         nextCameraKeyAction = new NextCameraKeyAction(allCommandsExecutor);
         cameraKeysMenu.add(nextCameraKeyAction);
      }

      if (previousCameraKeyAction == null)
      {
         previousCameraKeyAction = new PreviousCameraKeyAction(allCommandsExecutor);
         cameraKeysMenu.add(previousCameraKeyAction);
      }

      if (toggleCameraKeyModeAction == null)
      {
         toggleCameraKeyModeAction = new ToggleCameraKeyModeAction(allCommandsExecutor);
         cameraKeysMenu.add(toggleCameraKeyModeAction);
      }

      viewportMenu.add(viewMenu);

      if (extraPanelsMenu != null)
         viewportMenu.add(extraPanelsMenu);
      viewportMenu.add(cameraMenu);
      viewportMenu.add(cameraPropertiesAction);
      viewportMenu.add(hideShowViewportAction);

      viewportMenu.add(cameraKeysMenu);

      if (resizeViewportAction != null)
      {
         viewportMenu.add(resizeViewportAction);
      }
   }

   protected void setupCameraMenu(CameraConfigurationList cameraConfigurationList, CameraSelector cameraSelector)
   {
      cameraMenu.removeAll();
      String[] names = cameraConfigurationList.getCameraConfigurationNames();

      for (String name : names)
      {
         cameraMenu.add(new SelectCameraAction(cameraSelector, name));
      }
   }

   protected void setupExtraPanelsMenu(ExtraPanelConfigurationList extraPanelConfigurationList, ExtraPanelSelector panelSelector)
   {
      extraPanelsMenu.removeAll();
      String[] names = extraPanelConfigurationList.getExtraPanelConfigurationNames();
      for (String name : names)
      {
         JCheckBoxMenuItem myItem = new JCheckBoxMenuItem(new SelectExtraPanelAction(panelSelector, name));
         extraPanelsMenu.add(myItem);
      }
   }

   protected void updateVarGroupList(VarGroupList varGroupList, VarGroupSelector varGroupSelector)
   {
      varGroupsMenu.removeAll();

      String[] names = varGroupList.getVarGroupNames();

      for (int i = 0; i < names.length; i++)
      {
         varGroupsMenu.add(new SelectVarGroupAction(varGroupSelector, names[i]));
      }
   }

   @Override
   public void disableGUIComponents()
   {
      if (guiActions != null)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               for (Action action : guiActions)
               {
                  action.setEnabled(false);
               }
            }
         });
      }
   }

   @Override
   public void enableGUIComponents()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            if (guiActions != null)
            {
               for (Action action : guiActions)
               {
                  action.setEnabled(true);
               }
            }
         }
      });
   }

   protected void makeCheckBoxesConsistentWithCamera()
   {
      if (trackCheckBox != null) trackCheckBox.makeCheckBoxConsistent();
      if (dollyCheckBox != null) dollyCheckBox.makeCheckBoxConsistent();
   }

   protected void makeCameraConsistentWithCheckBoxes()
   {
      if (trackCheckBox != null) trackCheckBox.makeCameraConsistent();
      if (dollyCheckBox != null) dollyCheckBox.makeCameraConsistent();

   }

   protected void notifySimulationStopped()
   {
      EventDispatchThreadHelper.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            simulateAction.setEnabled(true);
            playAction.setEnabled(true);
         }
      });
   }

   public void setExportDataDirectory(String directory)
   {
      exportAction.setCurrentDirectory(directory);
   }

   public void loadGUIConfigurationFile(File file)
   {
      loadConfigurationAction.loadGUIConfigurationFile(file);
   }

   public void setImportDataDirectory(String directory)
   {
      importDataAction.setCurrentDirectory(directory);
   }

   public void createVideo(File file)
   {
      mediaCapture.createVideoFromFile(file);
   }

   public ArrayList<AbstractAction> getGuiActions()
   {
      return guiActions;
   }

   public void closeAndDispose()
   {
      if (guiActions != null)
      {
         guiActions.clear();
         guiActions = null;
      }

      openH264LicenseAction = null;
      aboutAction = null;

      if (cameraMenu != null)
      {
         cameraMenu.removeAll();
         cameraMenu = null;
      }

      if (viewportMenu != null)
      {
         viewportMenu.removeAll();
         viewportMenu = null;
      }

      if (viewMenu != null)
      {
         viewMenu.removeAll();
         viewMenu = null;
      }

      if (extraPanelsMenu != null)
      {
         extraPanelsMenu.removeAll();
         extraPanelsMenu = null;
      }

      if (cameraKeysMenu != null)
      {
         cameraKeysMenu.removeAll();
         cameraKeysMenu = null;
      }

      cameraPropertiesAction = null;

      if (configurationMenu != null)
      {
         configurationMenu.removeAll();
         configurationMenu = null;
      }

      createNewGraphWindowAction = null;

      createNewViewportWindowAction = null;

      cropBufferAction = null;
      packBufferAction = null;
      cutBufferAction = null;
      thinBufferAction = null;
      
      if (dataBufferPropertiesAction != null)
      {
         dataBufferPropertiesAction.closeAndDispose();
         dataBufferPropertiesAction = null;
      }

      dollyCheckBox = null;

      if (entryBoxGroupsMenu != null)
      {
         entryBoxGroupsMenu.removeAll();
         entryBoxGroupsMenu = null;
      }

      if (exitMenuItem != null)
      {
         exitMenuItem.removeAll();
         exitMenuItem = null;
      }

      if (exportAction != null)
      {
         exportAction.closeAndDispose();
         exportAction = null;
      }

      mediaCapture = null;

      exportSnapshotAction = null;

      goInPointAction = null;
      goOutPointAction = null;

      // protected EditVarGroupsAction editVarGroupsAction;

      if (graphGroupsMenu != null)
      {
         graphGroupsMenu.removeAll();
         graphGroupsMenu = null;
      }

      hideShowViewportAction = null;

      addCameraKeyAction = null;
      removeCameraKeyAction = null;
      nextCameraKeyAction = null;
      previousCameraKeyAction = null;

      if (importDataAction != null)
      {
         importDataAction.closeAndDispose();
         importDataAction = null;
      }

      playAction = null;
      playbackPropertiesAction = null;
      resizeViewportAction = null;

      if (runMenu != null)
      {
         runMenu.removeAll();
         runMenu = null;
      }

      setInPointAction = null;
      setKeyAction = null;
      setOutPointAction = null;

      saveConfigurationAction = null;
      saveGraphConfigurationAction = null;
      loadConfigurationAction = null;
      saveRobotConfigurationAction = null;
      exportSimulationTo3DMaxAction = null;

      // protected LoadRobotConfigurationAction loadRobotConfigurationAction;

      // protected SearchForVariableAction searchForVariableAction;
      simulateAction = null;
      stepBackwardAction = null;
      stepForwardAction = null;
      stopAction = null;
      toggleKeyPointModeAction = null;
      toggleCameraKeyModeAction = null;
      trackCheckBox = null;

      if (varGroupsMenu != null)
      {
         varGroupsMenu.removeAll();
         varGroupsMenu = null;
      }

      if (zoomInAction != null)
      {
         zoomInAction.closeAndDispose();
         zoomInAction = null;
      }

      if (zoomOutAction != null)
      {
         zoomOutAction.closeAndDispose();
         zoomOutAction = null;
      }

      if (printGraphsAction != null)
      {
         printGraphsAction.closeAndDispose();
         printGraphsAction = null;
      }
      
      if (exportGraphsToFileAction != null)
      {
         exportGraphsToFileAction.closeAndDispose();
         exportGraphsToFileAction = null;
      }

   }

}
