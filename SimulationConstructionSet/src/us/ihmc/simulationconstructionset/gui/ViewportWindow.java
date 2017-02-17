package us.ihmc.simulationconstructionset.gui;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.Toolkit;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JSplitPane;

import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfigurationList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.TrackingDollyCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.TimeHolder;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.commands.AllCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.DataBufferCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ExportSnapshotCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ExportVideoCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.gui.config.ViewportConfigurationList;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AllDialogConstructorsHolder;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.CameraPropertiesDialogGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ResizeViewportDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ResizeViewportDialogGenerator;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.videos.ExportVideo;

public class ViewportWindow implements ViewportSelectorCommandExecutor, ActiveCanvas3DHolder, ExtraPanelSelector
{
   private JFrame frame;
   private Container contentPane;
   private JPanel viewportJPanel;
   private ViewportSelectorCommandListener viewportSelectorCommandListener;
   private JSplitPane jsplitpane;

   private StandardGUIActions windowGUIActions;
   private StandardSimulationGUI myGUI;
   private ViewportConfigurationList viewportConfigurationList;
   private ViewportPanel viewportPanel;
   private JPanel buttonPanel;
   private ArrayList<Component> tempPanelsHolder = new ArrayList<Component>();
   private boolean isViewportHidden = false;

   private final String name;
   
   public ViewportWindow(AllCommandsExecutor allCommandsExecutor, YoVariableHolder yoVariableHolder, TimeHolder timeHolder, String selectedViewportName,
                         ViewportConfigurationList viewportConfigurationList, CameraConfigurationList cameraConfigurationList, CameraMountList cameraMountList,
                         Robot[] robots, VarGroupList varGroupList, GraphArrayPanel myGraphArrayPanel, StandardSimulationGUI myGUI,
                         Graphics3DAdapter graphics3DAdapter, DataBuffer dataBuffer,
                         StandardGUIActions mainGUIActions, int screenID, boolean maximizeWindow, SimulationSynchronizer simulationSynchronizer)
   {
      this.viewportConfigurationList = viewportConfigurationList;
      this.myGUI = myGUI;
      this.windowGUIActions = mainGUIActions;

      if (selectedViewportName != null)
         this.name = selectedViewportName;
      else this.name = "Unnamed";
      // this.varList = varList;
      // this.cameraMountList = cameraMountList;


      // GraphicsConfigTemplate3D graphicsConfigTemplate3D = new GraphicsConfigTemplate3D();
      // graphicsConfigTemplate3D.getBestConfiguration()

      GraphicsConfiguration configurationToUse = null;    // SimpleUniverse.getPreferredConfiguration(); //null;

      GraphicsEnvironment graphicsEnvironment = GraphicsEnvironment.getLocalGraphicsEnvironment();
      GraphicsDevice[] devices = graphicsEnvironment.getScreenDevices();
      for (int j = 0; j < devices.length; j++)
      {
         // GraphicsDevice graphicsDevice = devices[j];
         if (devices[j].toString().indexOf("screen=" + screenID) >= 0)
         {
            configurationToUse = devices[j].getDefaultConfiguration();
         }
      }

      frame = new JFrame("Viewport Window", configurationToUse);
      frame.setName("Viewport Window");
      contentPane = frame.getContentPane();

//    AllDialogConstructorsHolder allDialogConstructorsHolder = new StandardAllDialogConstructorsGenerator(sim, robots, dataBuffer, myGUI, varGroupList, myGraphArrayPanel, myGUI, frame, frame);
      windowGUIActions = new StandardGUIActions();

      viewportPanel = new ViewportPanel(yoVariableHolder, allCommandsExecutor, windowGUIActions, cameraConfigurationList, cameraMountList, graphics3DAdapter);

      ViewportConfiguration viewportConfig = viewportConfigurationList.getViewportConfiguration(selectedViewportName);

      viewportPanel.setupViews(frame.getGraphicsConfiguration().getDevice(), viewportConfig);


      GUIEnablerAndDisabler guiEnablerAndDisabler = allCommandsExecutor;
      RunCommandsExecutor runCommandsExecutor = allCommandsExecutor;
      DataBufferCommandsExecutor dataBufferCommandsExecutor = allCommandsExecutor;
      final ActiveCanvas3DHolder activeCanvas3DHolder = this;
      ViewportSelectorCommandExecutor viewportSelector = myGUI;

      ExportSnapshotCommandExecutor exportSnapshotCommandExecutor = new ExportSnapshotCommandExecutor()
      {
         @Override
         public void exportSnapshot(File snapshotFile)
         {
            CaptureDevice canvas3D = activeCanvas3DHolder.getActiveCaptureDevice();
            canvas3D.exportSnapshot(snapshotFile);
         }
      };

      ExportSnapshotDialogConstructor exportSnapshotDialogConstructor = new ExportSnapshotDialogGenerator(exportSnapshotCommandExecutor, guiEnablerAndDisabler,
                                                                           robots, activeCanvas3DHolder, frame);

      ExportVideoCommandExecutor exportVideoCommandExecutor = new ExportVideo(timeHolder, myGUI, dataBufferCommandsExecutor, runCommandsExecutor,
                                                                 guiEnablerAndDisabler, activeCanvas3DHolder, simulationSynchronizer);
      StopCommandExecutor stopCommandExecutor = allCommandsExecutor;

      MediaCaptureDialogConstructor mediaCaptureDialogConstructor = new MediaCaptureDialogGenerator(exportVideoCommandExecutor, guiEnablerAndDisabler,
                                                                       stopCommandExecutor, viewportSelector, myGUI, mainGUIActions, activeCanvas3DHolder);

      CameraPropertiesDialogGenerator cameraPropertiesDialogGenerator = new CameraPropertiesDialogGenerator(viewportPanel, frame, frame);
      ResizeViewportDialogConstructor resizeViewportDialogConstructor = new ResizeViewportDialogGenerator(frame, myGUI);

      windowGUIActions.createViewportWindowActions(mainGUIActions, exportSnapshotDialogConstructor, mediaCaptureDialogConstructor,
              cameraPropertiesDialogGenerator, resizeViewportDialogConstructor, viewportPanel, this);
      buttonPanel = windowGUIActions.createViewportWindowButtons();
      JMenuBar menuBar = windowGUIActions.createViewportWindowMenus();

      windowGUIActions.extraPanelsMenu = new JMenu("Extra Panels");
      windowGUIActions.setupExtraPanelsMenu(myGUI.getExtraPanelConfigurationList(), this);

      // windowGUIActions.setupViewportMenu(viewportConfigurationList);


      // contentPane.add(buttonPanel, "South");
      contentPane.setLayout(new BorderLayout());

      // contentPane.setLayout(new GridLayout(1,1));
      // contentPane.setLayout(new FlowLayout());
      // contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.Y_AXIS));
      // createActions();
      // setupConfigurationMenu();

      /*
       * viewportAndButtonPanel = new JPanel(new java.awt.BorderLayout());
       *    viewportAndButtonPanel.add("Center", viewportPanel);
       *    viewportAndButtonPanel.add("South", buttonPanel);
       *
       *    contentPane.add(viewportAndButtonPanel);
       */

      // contentPane.add(buttonPanel);
      // contentPane.add(myGraphArrayPanel);
      viewportJPanel = new JPanel(new BorderLayout());
      viewportJPanel.add(viewportPanel);

      contentPane.add(viewportJPanel);
      frame.setJMenuBar(menuBar);
      contentPane.add("South", buttonPanel);

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();

      // GraphicsDevice[] devices = GraphicsEnvironment.getLocalGraphicsEnvironment().getScreenDevices();
      // +++JEP: Add dual monitor support some day so a new frame can be born on the other monitor...
      frame.setSize(screenSize.width * 7 / 8, screenSize.height * 7 / 8);

      // frame.setSize(screenSize.width*1/8, screenSize.height*1/8);
      // frame.setLocation(screenSize.width/16, screenSize.height/16);
      frame.validate();

      // frame.pack();
      if (maximizeWindow)
      {
         frame.setExtendedState(Frame.MAXIMIZED_BOTH);
      }

      frame.setVisible(true);

      frame.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            for (Component panel : tempPanelsHolder)
            {
               makeCheckMarksConsistentForExtraPanels(panel.getName(), false);
            }
         }
      });

      // +++JEP: Why this was here I don't know, but it made things not show up
      // when you made a multi canvas viewport and showed it on a second window, only the first viewport would show up
      // ... this.selectViewport(selectedViewportName);
      myGUI.makeCheckMarksConsistentWithMainPanel(this);
   }

// For Robot Camera Vision:
   public ViewportWindow(AllCommandsExecutor allCommandsExecutor, AllDialogConstructorsHolder allDialogConstructorsHolder, SimulationConstructionSet sim,
                         Robot[] robots, TimeHolder timeHolder, YoVariableHolder yoVariableHolder, RunCommandsExecutor runCommandsExecutor,
                         GUIEnablerAndDisabler guiEnablerAndDisabler, String selectedViewportName, ViewportConfigurationList viewportConfigurationList,
                         CameraConfigurationList cameraConfigurationList, YoVariableList varList, CameraMountList cameraMountList, VarGroupList varGroupList,
                         GraphArrayPanel myGraphArrayPanel, AboutDialogGenerator aboutEditorPane, StandardSimulationGUI myGUI,
                         Graphics3DAdapter graphicsAdapter, DataBuffer dataBuffer,
                         StandardGUIActions mainGUIActions, int screenID, boolean maximizeWindow, SimulationSynchronizer simulationSynchronizer)
   {
      this.viewportConfigurationList = viewportConfigurationList;
      this.myGUI = myGUI;
      this.windowGUIActions = mainGUIActions;

      if (selectedViewportName != null)
         this.name = selectedViewportName;
      else this.name = "Unnamed";
      
//    this.varList = varList;
//    this.cameraMountList = cameraMountList;

//    GraphicsConfigTemplate3D graphicsConfigTemplate3D = new GraphicsConfigTemplate3D();
//    graphicsConfigTemplate3D.getBestConfiguration()

//G   raphicsConfiguration configurationToUse = null;    // SimpleUniverse.getPreferredConfiguration(); //null;
//

//G   raphicsEnvironment graphicsEnvironment = GraphicsEnvironment.getLocalGraphicsEnvironment();
//G   raphicsDevice[] devices = graphicsEnvironment.getScreenDevices();
//f   or (int j = 0; j < devices.length; j++)
//{
///   / GraphicsDevice graphicsDevice = devices[j];
//i   f (devices[j].toString().indexOf("screen=" + screenID) >= 0)
//{
//c   onfigurationToUse = devices[j].getDefaultConfiguration();
//}
//}

      frame = new JFrame("Viewport Window");
      frame.setName("Viewport Window");
      contentPane = frame.getContentPane();

      windowGUIActions = new StandardGUIActions();

      viewportPanel = new ViewportPanel(yoVariableHolder, runCommandsExecutor, windowGUIActions, cameraConfigurationList, cameraMountList, graphicsAdapter);

      ViewportConfiguration viewportConfig = viewportConfigurationList.getViewportConfiguration(selectedViewportName);

      viewportPanel.setupViews(frame.getGraphicsConfiguration().getDevice(), viewportConfig);


      DataBufferCommandsExecutor dataBufferCommandsExecutor = allCommandsExecutor;
      final ActiveCanvas3DHolder activeCanvas3DHolder = this;
      ViewportSelectorCommandExecutor viewportSelector = myGUI;

      ExportSnapshotCommandExecutor exportSnapshotCommandExecutor = new ExportSnapshotCommandExecutor()
      {
         @Override
         public void exportSnapshot(File snapshotFile)
         {
            CaptureDevice capturableCanvas = activeCanvas3DHolder.getActiveCaptureDevice();
            capturableCanvas.exportSnapshot(snapshotFile);
         }
      };
      ExportSnapshotDialogConstructor exportSnapshotDialogConstructor = new ExportSnapshotDialogGenerator(exportSnapshotCommandExecutor, guiEnablerAndDisabler,
                                                                           robots, activeCanvas3DHolder, frame);

      ExportVideoCommandExecutor exportVideoCommandExecutor = new ExportVideo(timeHolder, myGUI, dataBufferCommandsExecutor, runCommandsExecutor,
                                                                 guiEnablerAndDisabler, activeCanvas3DHolder, simulationSynchronizer);
      StopCommandExecutor stopCommandExecutor = sim;

      MediaCaptureDialogConstructor mediaCaptureDialogConstructor = new MediaCaptureDialogGenerator(exportVideoCommandExecutor, guiEnablerAndDisabler,
                                                                       stopCommandExecutor, viewportSelector, myGUI, mainGUIActions, activeCanvas3DHolder);
      CameraPropertiesDialogGenerator cameraPropertiesDialogGenerator = new CameraPropertiesDialogGenerator(viewportPanel, frame, frame);
      ResizeViewportDialogConstructor resizeViewportDialogConstructor = new ResizeViewportDialogGenerator(frame, myGUI);

      windowGUIActions.createViewportWindowActions(mainGUIActions, exportSnapshotDialogConstructor, mediaCaptureDialogConstructor,
              cameraPropertiesDialogGenerator, resizeViewportDialogConstructor, viewportPanel, this);
      buttonPanel = windowGUIActions.createViewportWindowButtons();
      JMenuBar menuBar = windowGUIActions.createViewportWindowMenus();

      windowGUIActions.extraPanelsMenu = new JMenu("Extra Panels");
      windowGUIActions.setupExtraPanelsMenu(myGUI.getExtraPanelConfigurationList(), this);

//    windowGUIActions.setupViewportMenu(viewportConfigurationList);


//    contentPane.add(buttonPanel, "South");
      contentPane.setLayout(new BorderLayout());

//    contentPane.setLayout(new GridLayout(1,1));
//    contentPane.setLayout(new FlowLayout());
//    contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.Y_AXIS));
//    createActions();
//    setupConfigurationMenu();

/*
      * viewportAndButtonPanel = new JPanel(new java.awt.BorderLayout());
      *    viewportAndButtonPanel.add("Center", viewportPanel);
      *    viewportAndButtonPanel.add("South", buttonPanel);
      *
      *    contentPane.add(viewportAndButtonPanel);
*/

//    contentPane.add(buttonPanel);
//    contentPane.add(myGraphArrayPanel);
      viewportJPanel = new JPanel(new BorderLayout());

      viewportJPanel.add(viewportPanel);

      contentPane.add(viewportJPanel);
      frame.setJMenuBar(menuBar);
      contentPane.add("South", buttonPanel);

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();

//    GraphicsDevice[] devices = GraphicsEnvironment.getLocalGraphicsEnvironment().getScreenDevices();
//    +++JEP: Add dual monitor support some day so a new frame can be born on the other monitor...
      frame.setSize(screenSize.width * 7 / 8, screenSize.height * 7 / 8);

//    frame.setSize(screenSize.width*1/8, screenSize.height*1/8);
//    frame.setLocation(screenSize.width/16, screenSize.height/16);
      frame.validate();

//    frame.pack();
      if (maximizeWindow)
      {
         frame.setExtendedState(Frame.MAXIMIZED_BOTH);
      }

      frame.setVisible(true);

      frame.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            for (Component panel : tempPanelsHolder)
            {
               makeCheckMarksConsistentForExtraPanels(panel.getName(), false);
            }
         }
      });

//    +++JEP: Why this was here I don't know, but it made things not show up
//    when you made a multi canvas viewport and showed it on a second window, only the first viewport would show up
//    ... this.selectViewport(selectedViewportName);
      myGUI.makeCheckMarksConsistentWithMainPanel(this);
   }

   public String getName()
   {
      return name;
   }

   public void makeExtraPanelsMenuConsistent(String panelName, boolean isSelected)
   {
      for (int i = 0; i < windowGUIActions.extraPanelsMenu.getItemCount(); i++)
      {
         if (panelName.equals(windowGUIActions.extraPanelsMenu.getItem(i).getText()))
         {
            windowGUIActions.extraPanelsMenu.getItem(i).setSelected(isSelected);
         }
      }
   }

   public boolean isSelectedPanel(String panelName)
   {
      boolean isSelected = false;
      for (int i = 0; i < windowGUIActions.extraPanelsMenu.getItemCount(); i++)
      {
         if (panelName.equals(windowGUIActions.extraPanelsMenu.getItem(i).getText()) && windowGUIActions.extraPanelsMenu.getItem(i).isSelected())
         {
            isSelected = true;
         }
      }

      return isSelected;
   }


   public void removeExtraPanel(String panelName)
   {
      tempPanelsHolder.remove(myGUI.getExtraPanel(panelName));
      drawViewportWithExtraPanels();
   }


   public String savingExtraPanels()
   {
      String textToWrite = "";
      for (Component name : tempPanelsHolder)
      {
         textToWrite += name.getName() + ",";
      }

      return textToWrite;
   }

   public void addPanelToTempHolder(Component extraPanel)
   {
      tempPanelsHolder.add(extraPanel);
      drawViewportWithExtraPanels();
   }

   private void drawViewportWithExtraPanels()
   {
      if (viewportJPanel != null)
         viewportJPanel.removeAll();

      if (jsplitpane != null)
         jsplitpane.removeAll();

      if (tempPanelsHolder.size() == 0)
      {
         viewportJPanel.add(viewportPanel);
      }
      else
      {
         ArrayList<JSplitPane> dividers = new ArrayList<JSplitPane>();
         jsplitpane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, viewportPanel, tempPanelsHolder.get(0));
         dividers.add(jsplitpane);

         for (int i = 1; i < tempPanelsHolder.size(); i++)
         {
            jsplitpane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, jsplitpane, tempPanelsHolder.get(i));
            dividers.add(jsplitpane);
         }

         int currentPlaceOfLastDivider = 0;
         for (int j = 0; j < dividers.size(); j++)
         {
            dividers.get(j).setDividerLocation(currentPlaceOfLastDivider + viewportJPanel.getWidth() / ((dividers.size() + 1)));
            currentPlaceOfLastDivider += viewportJPanel.getWidth() / ((dividers.size() + 1));
         }

         viewportJPanel.add(jsplitpane);
      }

      // This is to prevent the viewports from not being drawing
      int width = frame.getWidth() + 1;
      int height = frame.getHeight() + 1;
      Dimension d = new Dimension(width, height);

      frame.setSize(d);
      viewportPanel.repaint();
      viewportJPanel.updateUI();
   }

   public void makeCheckMarksConsistentForExtraPanels(String panelName, boolean isSelected)
   {
      myGUI.makeCheckMarksConsistentForExtraPanels(panelName, isSelected);
   }

   @Override
   public void selectPanel(String panelName)
   {
      if (isSelectedPanel(panelName))
      {
         makeCheckMarksConsistentForExtraPanels(panelName, true);
         tempPanelsHolder.add(myGUI.getExtraPanel(panelName));
         drawViewportWithExtraPanels();
      }
      else
      {
         boolean wasInThisWindow = true;
         if (!tempPanelsHolder.contains(myGUI.getExtraPanel(panelName)))
         {
            wasInThisWindow = false;
         }

         makeCheckMarksConsistentForExtraPanels(panelName, false);
         myGUI.removeExtraPanel(panelName);

         if (!wasInThisWindow)
         {
            tempPanelsHolder.add(myGUI.getExtraPanel(panelName));
            drawViewportWithExtraPanels();
         }
      }
   }




   public ViewportPanel getViewportPanel()
   {
      return this.viewportPanel;
   }

   public CameraPropertiesHolder getActiveCamera()
   {
      return viewportPanel.getCameraPropertiesForActiveCamera();
   }

   public ViewportAdapter getActiveView()
   {
      return viewportPanel.getActiveView();
   }

   public ArrayList<ViewportAdapterAndCameraControllerHolder> getCameraAdapters()
   {
      return viewportPanel.getCameraAdapters();
   }

   @Override
   public TrackingDollyCameraController getCamera()
   {
      return viewportPanel.getCamera();
   }

   @Override
   public CaptureDevice getActiveCaptureDevice()
   {
      return this.getActiveView().getCaptureDevice();
   }

   public StandardGUIActions getGUIActions()
   {
      return this.windowGUIActions;
   }

   public boolean isVisable()
   {
      return frame.isVisible();
   }

   @Override
   public void selectViewport(String viewportName)
   {
      ViewportConfiguration viewportConfiguration = viewportConfigurationList.getViewportConfiguration(viewportName);

      if (viewportConfiguration == null)
      {
         return;
      }

      viewportPanel.setupViews(frame.getGraphicsConfiguration().getDevice(), viewportConfiguration);
      viewportPanel.updateUI();
      makeCheckBoxesConsistentWithCamera();
   }

   public void makeCheckBoxesConsistentWithCamera()
   {
      windowGUIActions.makeCheckBoxesConsistentWithCamera();
   }

   @Override
   public void showViewport()
   {
      isViewportHidden = false;
      contentPane.removeAll();
      contentPane.add(viewportPanel);
      contentPane.add("South", buttonPanel);
      viewportSelectorCommandListener.updateViewportStatus();
      viewportPanel.updateUI();
   }

   @Override
   public void hideViewport()
   {
      isViewportHidden = true;
      contentPane.removeAll();
      contentPane.add("South", buttonPanel);
      viewportSelectorCommandListener.updateViewportStatus();
      viewportPanel.updateUI();
   }

   public void closeWindow()
   {
      frame.setVisible(false);
   }

   @Override
   public boolean isViewportHidden()
   {
      return isViewportHidden;
   }

   @Override
   public void registerViewportSelectorCommandListener(ViewportSelectorCommandListener viewportSelectorCommandListener)
   {
      this.viewportSelectorCommandListener = viewportSelectorCommandListener;
   }

   public String getMainViewPortPanelXML(boolean visible_ViewPort)
   {
      return viewportPanel.getXMLStyleRepresentationOfMainViewPort(visible_ViewPort);
   }

   public String getXMLStyleRepresentationOfClass(ViewportAdapterAndCameraControllerHolder view3d, int canvasNumber)
   {
      return viewportPanel.getXMLStyleRepresentationOfClassViewPorts(view3d, canvasNumber);
   }

   @Override
   public void closeAndDispose()
   {
      viewportPanel.closeAndDispose();
      
      if (windowGUIActions != null)
      {
         windowGUIActions.closeAndDispose();
         windowGUIActions = null;
      }

      frame.setMenuBar(null);
      frame.removeAll();

      frame.dispose();

      windowGUIActions = null;
   }

}
