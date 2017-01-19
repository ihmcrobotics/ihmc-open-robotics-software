package us.ihmc.simulationconstructionset.gui;

import java.awt.Dimension;
import java.awt.GraphicsDevice;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Tuple3d;

import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfigurationList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountList;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.TrackingDollyCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.ViewportPanelConfiguration;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.gui.camera.CameraTrackAndDollyYoVariablesHolder;
import us.ihmc.simulationconstructionset.gui.config.CameraSelector;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class ViewportPanel extends JPanel implements CameraSelector, ActiveCameraHolder, ActiveCanvas3DHolder
{
   private static final long serialVersionUID = -2903057563160516887L;
   private static final boolean DEBUG_CLOSE_AND_DISPOSE = false;
   private ArrayList<ViewportAdapterAndCameraControllerHolder> standard3DViews = new ArrayList<ViewportAdapterAndCameraControllerHolder>();
   private ArrayList<Canvas3DPanel> canvasPanels = new ArrayList<Canvas3DPanel>();
   private ViewportAdapterAndCameraControllerHolder activeView;
   private CameraConfigurationList cameraConfigurationList;
   private CameraMountList cameraMountList;
   private YoVariableHolder yoVariableHolder;
   private RunCommandsExecutor runCommandsExecutor;
   private Graphics3DAdapter graphics3DAdapter;

   private StandardGUIActions standardGUIActions;

   public ViewportPanel(YoVariableHolder yoVariableHolder, RunCommandsExecutor runCommandsExecutor, StandardGUIActions standardGUIActions,
         CameraConfigurationList cameraConfigurationList, CameraMountList cameraMountList, Graphics3DAdapter graphics3DAdapater)
   {
      this.yoVariableHolder = yoVariableHolder;
      this.runCommandsExecutor = runCommandsExecutor;
      this.standardGUIActions = standardGUIActions;
      this.cameraConfigurationList = cameraConfigurationList;
      this.cameraMountList = cameraMountList;
      this.graphics3DAdapter = graphics3DAdapater;

   }

   @Override
   public TrackingDollyCameraController getCameraPropertiesForActiveCamera()
   {
      return activeView.getCameraController();
   }
   
   public void setupViews(GraphicsDevice graphicsDevice, ViewportConfiguration viewportConfig)
   {
      setupViews(graphicsDevice, viewportConfig, null);
   }

   public void setupViews(GraphicsDevice graphicsDevice, ViewportConfiguration viewportConfig, JFrame jFrame)
   {
      this.removeAll();

      clearStandard3DViews();
      canvasPanels.clear();

      if (viewportConfig == null)
      {
         this.setLayout(new GridLayout(1, 1));

         CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyYoVariablesHolder = new CameraTrackAndDollyYoVariablesHolder(yoVariableHolder);

         ViewportAdapter standard3DView = graphics3DAdapter.createNewViewport(graphicsDevice, false, false);
         
         ClassicCameraController classicCameraController;
         if (jFrame != null)
            classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(standard3DView, cameraTrackAndDollyYoVariablesHolder, graphics3DAdapter, jFrame);
         else
            classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(standard3DView, cameraTrackAndDollyYoVariablesHolder, graphics3DAdapter);  
         
         ViewportAdapterAndCameraControllerHolder viewportAdapterAndCameraControllerHolder = new ViewportAdapterAndCameraControllerHolder(standard3DView, classicCameraController);
         standard3DView.setCameraController(classicCameraController);

         standard3DViews.add(viewportAdapterAndCameraControllerHolder);
         activeView = viewportAdapterAndCameraControllerHolder;

         Canvas3DPanel panel_j = new Canvas3DPanel(runCommandsExecutor, viewportAdapterAndCameraControllerHolder, this);

         this.add(panel_j);

         return;
      }

      ArrayList<ViewportPanelConfiguration> panelConfigs = viewportConfig.getPanelConfigurations();

      GridBagLayout gridBag = new GridBagLayout();
      this.setLayout(gridBag);

      GridBagConstraints c;

      for (int i = 0; i < panelConfigs.size(); i++)
      {
         c = new GridBagConstraints();
         ViewportPanelConfiguration panelConfiguration = (ViewportPanelConfiguration) panelConfigs.get(i);

         CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyYoVariablesHolder = new CameraTrackAndDollyYoVariablesHolder(yoVariableHolder);

         ViewportAdapter standard3DView = graphics3DAdapter.createNewViewport(graphicsDevice, false, false);
         ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(standard3DView,
               cameraTrackAndDollyYoVariablesHolder, graphics3DAdapter);
         ViewportAdapterAndCameraControllerHolder viewportAdapterAndCameraControllerHolder = new ViewportAdapterAndCameraControllerHolder(standard3DView,
               classicCameraController);
         standard3DView.setCameraController(classicCameraController);

         standard3DViews.add(viewportAdapterAndCameraControllerHolder);

         CameraConfiguration cameraConfig = cameraConfigurationList.getCameraConfiguration(panelConfiguration.cameraName);

         if (cameraConfig != null)
         {
            setCameraConfiguration(viewportAdapterAndCameraControllerHolder.getCameraController(), cameraConfig, yoVariableHolder, cameraMountList);
         }
         else
         {
            System.err.println("Warning. No camera named " + panelConfiguration.cameraName);

            continue;
         }

         Canvas3DPanel panel_j = new Canvas3DPanel(runCommandsExecutor, viewportAdapterAndCameraControllerHolder, this);
         canvasPanels.add(panel_j);

         c.gridx = panelConfiguration.gridx;
         c.gridy = panelConfiguration.gridy;
         c.gridwidth = panelConfiguration.gridwidth;
         c.gridheight = panelConfiguration.gridheight;
         c.weightx = panelConfiguration.gridwidth;
         c.weighty = panelConfiguration.gridheight;
         // c.weightx = 0.5;
         // c.weighty = 0.5; 
         // c.anchor = GridBagConstraints.CENTER;
         c.fill = GridBagConstraints.BOTH;

         gridBag.setConstraints(panel_j, c);
         
         this.add(panel_j);
         
         if (i == 0)
         {
            activeView = viewportAdapterAndCameraControllerHolder;
            panel_j.setActive(true);
         }
         else
         {
            panel_j.setActive(false);
         }
      }
   }

   /*
    * public JPanel setupViewsCaching(GraphicsDevice graphicsDevice,
    * ViewportConfiguration viewportConfig, Locale locale, PreRenderer
    * preRenderer, ViewportPanelUpdateBehavior viewportPanelUpdateBehavior) {
    * JPanel panel = null;
    * 
    * if (viewportConfig != null) panel =
    * viewportConfig.getCachedViewportPanel(); if (panel != null)return panel;
    * 
    * if (panel == null) panel = new JPanel();
    * 
    * // Remove all from the locale: for (Standard3DView standard3DView :
    * standard3DViews) //++++++ { //standard3DView.detachAll();
    * //standard3DView.detach();
    * //++++++locale.removeBranchGraph(standard3DView); }
    * 
    * panel.removeAll(); //standard3DViews.clear(); clearStandard3DViews();
    * canvasPanels.clear();
    * 
    * //standard3DViews = new ArrayList(); //canvasPanels = new ArrayList();
    * 
    * // "Force" Garbage Collection: System.gc();
    * 
    * if (viewportConfig == null) { panel.setLayout(new GridLayout(1, 1));
    * Standard3DView standard3DView = new Standard3DView(graphicsDevice,
    * navigatingCameraHolder, rob, locale, preRenderer);
    * standard3DViews.add(standard3DView); activeView = standard3DView;
    * navigatingCameraHolder.setNavigatingCamera(activeView.getCamera());
    * 
    * locale.addBranchGraph(standard3DView); YoCanvas3D canvas =
    * standard3DView.getCanvas3D();
    * 
    * //panel.add(canvas);
    * 
    * Canvas3DPanel panel_j = new Canvas3DPanel(canvas, standard3DView, this);
    * panel.add(panel_j);
    * 
    * viewportPanelUpdateBehavior.attachViewportPanel(this);
    * 
    * 
    * return panel; }
    * 
    * //ArrayList cachedViews = viewportConfig.getCached3DViews(); // if
    * (cachedViews != null) // { // for(int i=0; i<cachedViews.size(); i++) // {
    * // Standard3DView cachedView = (Standard3DView) cachedViews.get(i); //
    * locale.addBranchGraph(cachedView); // } // return; // }
    * 
    * //CameraConfiguration[][] configs = windowConfig.getCameraLayout();
    * 
    * ArrayList panelConfigs = viewportConfig.getPanelConfigurations();
    * 
    * //activeView = null;
    * 
    * GridBagLayout gridBag = new GridBagLayout(); GridBagConstraints c = new
    * GridBagConstraints(); panel.setLayout(gridBag); c.fill =
    * GridBagConstraints.BOTH;
    * 
    * ArrayList cachedViews = viewportConfig.getCached3DViews(); ArrayList
    * cachedPanels = viewportConfig.getCachedCanvas3DPanels();
    * 
    * for (int i = 0; i < panelConfigs.size(); i++) { ViewportPanelConfiguration
    * panelConfiguration = (ViewportPanelConfiguration) panelConfigs.get(i);
    * 
    * Standard3DView standard3DView;
    * 
    * if (cachedViews != null) standard3DView = (Standard3DView)
    * cachedViews.get(i); else standard3DView = new
    * Standard3DView(graphicsDevice, navigatingCameraHolder, rob, locale,
    * preRenderer);
    * 
    * standard3DViews.add(standard3DView);
    * 
    * if (cachedViews == null) locale.addBranchGraph(standard3DView); //++++++
    * Only need to add to the locale once. Otherwise keep it there.
    * CameraConfiguration cameraConfig =
    * cameraConfigurationList.getCameraConfiguration
    * (panelConfiguration.cameraName);
    * 
    * standard3DView.setCameraConfiguration(cameraConfig, allVariables,
    * cameraMountList);
    * 
    * YoCanvas3D canvas = standard3DView.getCanvas3D();
    * 
    * Canvas3DPanel panel_j; // = new Canvas3DPanel(canvas, standard3DView,
    * this);
    * 
    * if (cachedPanels != null) { panel_j = (Canvas3DPanel) cachedPanels.get(i);
    * panel_j.setCanvasAndView(canvas, standard3DView); } else panel_j = new
    * Canvas3DPanel(canvas, standard3DView, this);
    * 
    * canvasPanels.add(panel_j); //standard3DView.setPanel(panel_j);
    * 
    * c.gridx = panelConfiguration.gridx; c.gridy = panelConfiguration.gridy;
    * c.gridwidth = panelConfiguration.gridwidth; c.gridheight =
    * panelConfiguration.gridheight; c.weightx = 0.5; //++++++ c.weighty = 0.5;
    * c.anchor = c.CENTER;
    * 
    * gridBag.setConstraints(panel_j, c);
    * 
    * panel.add(panel_j);
    * 
    * //if (activeView == null) if (i == 0) { activeView = standard3DView;
    * navigatingCameraHolder.setNavigatingCamera(activeView.getCamera());
    * 
    * panel_j.setActive(true); //activeView.setActive(true); } else {
    * panel_j.setActive(false); } }
    * 
    * viewportConfig.cache3DViews(standard3DViews);
    * viewportConfig.cacheCanvas3DPanels(canvasPanels);
    * 
    * viewportConfig.cacheViewportPanel(panel);
    * 
    * viewportPanelUpdateBehavior.attachViewportPanel(this); return panel; }
    */
   public ViewportAdapter getActiveView()
   {
      return this.activeView.getViewportAdapter();
   }

   public void setActiveView(ViewportAdapterAndCameraControllerHolder view, Canvas3DPanel activePanel)
   {
      if (this.activeView == view)
      {
         return;
      }

      view.getCameraController().reset(); // Reset camera so drags are forgotten and do not move the new cameras fix/pos.
      this.activeView = view;

      for (Canvas3DPanel canvas3DPanel : canvasPanels)
      {
         canvas3DPanel.setActive(false);
      }

      // activeView.setActive(true);
      activePanel.setActive(true);

      if (standardGUIActions != null)
      {
         standardGUIActions.makeCheckBoxesConsistentWithCamera();
      }
   }

   public StandardGUIActions getStandardGUIActions()
   {
      return standardGUIActions;
   }

   public ArrayList<ViewportAdapterAndCameraControllerHolder> getCameraAdapters()
   {
      return this.standard3DViews;
   }

   public void closeAndDispose()
   {
      if (DEBUG_CLOSE_AND_DISPOSE) System.out.println("Closing and Disposing things in " + getClass().getSimpleName());
      
      if (standard3DViews != null)
      {
         clearStandard3DViews();
         standard3DViews = null;
      }

      if (canvasPanels != null)
      {
         for (Canvas3DPanel canvas3dPanel : canvasPanels)
         {
            canvas3dPanel.closeAndDispose();
         }
         
         canvasPanels.clear();
         canvasPanels = null;
      }

      activeView = null;

      cameraConfigurationList = null;
      cameraMountList = null;
      yoVariableHolder = null;
      standardGUIActions = null;
      runCommandsExecutor = null;
      graphics3DAdapter = null;
   }

   // public YoCanvas3D getCanvas3D(){return activeView.getCanvas3D();}
   // public YoCanvas3D getOffscreenCanvas3D(){return this.offscreenCanvas3D;}
   @Override
   public TrackingDollyCameraController getCamera()
   {
      return activeView.getCameraController();
   }

   public TrackingDollyCameraController[] getCameras()
   {
      TrackingDollyCameraController[] cameras = new TrackingDollyCameraController[standard3DViews.size()];
      for (int i = 0; i < standard3DViews.size(); i++)
      {
         cameras[i] = standard3DViews.get(i).getCameraController();
      }

      return cameras;
   }

   public void setClipDistances(double near, double far)
   {
      activeView.getCameraController().setClipDistanceNear(near);
      activeView.getCameraController().setClipDistanceFar(far);
   }

   public void setFieldOfView(double fieldOfView)
   {
      activeView.getCameraController().setFieldOfView(fieldOfView);
   }

   public void setCameraTrackingVars(DoubleYoVariable xVar, DoubleYoVariable yVar, DoubleYoVariable zVar)
   {
      CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyVariablesHolder = (CameraTrackAndDollyYoVariablesHolder) activeView.getCameraController()
            .getCameraTrackAndDollyVariablesHolder();
      cameraTrackAndDollyVariablesHolder.setTrackingVars(xVar, yVar, zVar);

      //    activeView.getCamera().setTrackingVars(xVar, yVar, zVar);
   }

   public void setCameraDollyVars(DoubleYoVariable xVar, DoubleYoVariable yVar, DoubleYoVariable zVar)
   {
      CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyVariablesHolder = (CameraTrackAndDollyYoVariablesHolder) activeView.getCameraController()
            .getCameraTrackAndDollyVariablesHolder();
      cameraTrackAndDollyVariablesHolder.setDollyVars(xVar, yVar, zVar);

      //    activeView.getCamera().setDollyVars(xVar, yVar, zVar);
   }

   public void setCameraTrackingOffsets(double dx, double dy, double dz)
   {
      activeView.getCameraController().setTrackingOffsets(dx, dy, dz);
   }

   public void setCameraDollyOffsets(double dx, double dy, double dz)
   {
      activeView.getCameraController().setDollyOffsets(dx, dy, dz);
   }

   public void setCameraFix(double fixX, double fixY, double fixZ)
   {
      activeView.getCameraController().setFixPosition(fixX, fixY, fixZ);
   }

   public void setCameraFix(Tuple3d cameraFix)
   {
      activeView.getCameraController().setFixPosition(cameraFix.getX(), cameraFix.getY(), cameraFix.getZ());      
   }

   public void setCameraPosition(double posX, double posY, double posZ)
   {
      activeView.getCameraController().setCameraPosition(posX, posY, posZ);
   }

   public void setCameraPosition(Tuple3d cameraPosition)
   {
      activeView.getCameraController().setCameraPosition(cameraPosition.getX(), cameraPosition.getY(), cameraPosition.getZ());      
   }

   public void setCameraTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
   {
      activeView.getCameraController().setTracking(track, trackX, trackY, trackZ);
   }

   public void setCameraDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
   {
      activeView.getCameraController().setDolly(dolly, dollyX, dollyY, dollyZ);
   }

   public void setCameraConfiguration(CameraConfiguration config, YoVariableHolder holder)
   {
      setCameraConfiguration(activeView.getCameraController(), config, holder, cameraMountList);
   }

   // public void updateGroundDisplay()
   // {
   //    standardGUIActions.updateGroundDisplay();
   // }

   public synchronized void clearStandard3DViews()
   {
      for (ViewportAdapterAndCameraControllerHolder viewportAdapterAndCameraControllerHolder : standard3DViews)
      {
         graphics3DAdapter.closeViewport(viewportAdapterAndCameraControllerHolder.getViewportAdapter());
         viewportAdapterAndCameraControllerHolder.closeAndDispose();
      }

      standard3DViews.clear();
   }

   @Override
   public CaptureDevice getActiveCaptureDevice()
   {
      return this.getActiveView().getCaptureDevice();
   }

   public void selectActiveCanvas3D(int canvasIndex)
   {
      if (canvasIndex >= canvasPanels.size())
      {
         return;
      }

      Canvas3DPanel canvasPanel = this.canvasPanels.get(canvasIndex);

      setActiveView(canvasPanel.getStandard3DView(), canvasPanel);
   }

   @Override
   public void selectCamera(String name)
   {
      CameraConfiguration config = cameraConfigurationList.getCameraConfiguration(name);

      if (config == null)
      {
         return;
      }

      setCameraConfiguration(config, yoVariableHolder); // , cameraMountList);

      //    makeCheckBoxesConsistentWithCamera();
   }

   public String getXMLStyleRepresentationofMultiViews(String currentView)
   {
      String textToWrite = "";
      textToWrite += "<Current View>" + currentView + "</Current View>";

      for (int i = 0; i < canvasPanels.size(); i++)
      {
         int label = i + 1;
         textToWrite += "\n<Canvas " + label + ">";
         textToWrite += "\n<Camera X>" + canvasPanels.get(i).getStandard3DView().getCameraController().getCamX() + "</Camera X>";
         textToWrite += "\n<Camera Y>" + canvasPanels.get(i).getStandard3DView().getCameraController().getCamY() + "</Camera Y>";
         textToWrite += "\n<Camera Z>" + canvasPanels.get(i).getStandard3DView().getCameraController().getCamZ() + "</Camera Z>";
         textToWrite += "\n <Dolly data> \n <Position X>" + canvasPanels.get(i).getStandard3DView().getCameraController().getDollyXOffset() + "</Position X>"
               + "\n <Position Y>" + canvasPanels.get(i).getStandard3DView().getCameraController().getDollyYOffset() + "</Position Y>" + "\n <Position Z>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().getDollyZOffset() + "</Position Z>";
         textToWrite += "\n<Dolly Booleans>" + "\n<Dolly>" + canvasPanels.get(i).getStandard3DView().getCameraController().isDolly() + "</Dolly>"
               + "\n<Dolly X>" + canvasPanels.get(i).getStandard3DView().getCameraController().isDollyX() + "</Dolly X>" + "\n<Dolly Y>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().isDollyY() + "</Dolly Y>" + "\n<Dolly Z>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().isDollyY() + "</Dolly Z>" + "\n</Dolly Booleans>";
         textToWrite += "\n</Dolly data>";
         textToWrite += "\n <Track data> \n <Position X>" + canvasPanels.get(i).getStandard3DView().getCameraController().getTrackingXOffset()
               + "</Position X>" + "\n <Position Y>" + canvasPanels.get(i).getStandard3DView().getCameraController().getTrackingYOffset() + "</Position Y>"
               + "\n <Position Z>" + canvasPanels.get(i).getStandard3DView().getCameraController().getTrackingZOffset() + "</Position Z>";
         textToWrite += "\n <Track Booleans>" + "\n<Track>" + canvasPanels.get(i).getStandard3DView().getCameraController().isTracking() + "</Track>"
               + "\n<Track X>" + canvasPanels.get(i).getStandard3DView().getCameraController().isTrackingX() + "</Track X>" + "\n<Track Y>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().isTrackingY() + "</Track Y>" + "\n<Track Z>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().isTracking() + "</Track Z>" + "\n</Track Booleans>";
         textToWrite += "\n</Track data>";
         textToWrite += "\n<Fix Position>" + "\n<Fix X>" + canvasPanels.get(i).getStandard3DView().getCameraController().getFixX() + "</Fix X>" + "\n<Fix Y>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().getFixY() + "</Fix Y>" + "\n<Fix Z>"
               + canvasPanels.get(i).getStandard3DView().getCameraController().getFixZ() + "</Fix Z>" + "\n</Fix Position>";
         textToWrite += "\n</Canvas " + label + ">";
      }

      if (canvasPanels.size() == 0)
      {
         textToWrite += "\n <Canvas 1> " + "\n <Camera X>" + getCamera().getCamX() + "</Camera X>" + "\n <Camera Y>" + getCamera().getCamY() + "</Camera Y>"
               + "\n <Camera Z>" + getCamera().getCamZ() + "</Camera Z>";
         textToWrite += "\n <Dolly data> \n <Position X>" + getCameraPropertiesForActiveCamera().getDollyXOffset() + "</Position X>" + "\n <Position Y>"
               + getCameraPropertiesForActiveCamera().getDollyYOffset() + "</Position Y>" + "\n <Position Z>"
               + getCameraPropertiesForActiveCamera().getDollyZOffset() + "</Position Z>";
         textToWrite += "\n<Dolly Booleans>" + "\n<Dolly>" + getCameraPropertiesForActiveCamera().isDolly() + "</Dolly>" + "\n<Dolly X>"
               + getCameraPropertiesForActiveCamera().isDollyX() + "</Dolly X>" + "\n<Dolly Y>" + getCameraPropertiesForActiveCamera().isDollyY()
               + "</Dolly Y>" + "\n<Dolly Z>" + getCameraPropertiesForActiveCamera().isDollyY() + "</Dolly Z>" + "\n</Dolly Booleans>";
         textToWrite += "\n</Dolly data>";
         textToWrite += "\n <Track data> \n <Position X>" + getCameraPropertiesForActiveCamera().getTrackingXOffset() + "</Position X>" + "\n <Position Y>"
               + getCameraPropertiesForActiveCamera().getTrackingYOffset() + "</Position Y>" + "\n <Position Z>"
               + getCameraPropertiesForActiveCamera().getTrackingZOffset() + "</Position Z>";
         textToWrite += "\n <Track Booleans>" + "\n<Track>" + getCameraPropertiesForActiveCamera().isTracking() + "</Track>" + "\n<Track X>"
               + getCameraPropertiesForActiveCamera().isTrackingX() + "</Track X>" + "\n<Track Y>" + getCameraPropertiesForActiveCamera().isTrackingY()
               + "</Track Y>" + "\n<Track Z>" + getCameraPropertiesForActiveCamera().isTracking() + "</Track Z>" + "\n</Track Booleans>";
         textToWrite += "\n</Track data>";
         textToWrite += "\n<Fix Position>" + "\n<Fix X>" + getCameraPropertiesForActiveCamera().getFixX() + "</Fix X>" + "\n<Fix Y>"
               + getCameraPropertiesForActiveCamera().getFixY() + "</Fix Y>" + "\n<Fix Z>" + getCameraPropertiesForActiveCamera().getFixZ() + "</Fix Z>"
               + "\n</Fix Position>";
         textToWrite += "\n</Canvas 1>";
      }

      return textToWrite;
   }

   public String getXMLStyleRepresentationOfMainViewPort(boolean visible_ViewPort)
   {
      String textToWrite = "";

      textToWrite += "\n <Main Viewport> " + "\n<Visible> " + visible_ViewPort + "</Visible>";
      textToWrite += "\n</Main Viewport>";

      return textToWrite;
   }

   public String getXMLStyleRepresentationOfClassViewPorts(ViewportAdapterAndCameraControllerHolder view3d, int canvasNumber)
   {
      String textToWrite = "";
      textToWrite += "\n<Canvas" + canvasNumber + ">";
      textToWrite += "\n<Viewport Camera X>" + view3d.getCameraController().getCamX() + "</Viewport Camera X>";
      textToWrite += "\n<Viewport Camera Y>" + view3d.getCameraController().getCamY() + "</Viewport Camera Y>";
      textToWrite += "\n<Viewport Camera Z>" + view3d.getCameraController().getCamZ() + "</Viewport Camera Z>";
      textToWrite += "\n <Dolly data> \n <Position X>" + view3d.getCameraController().getDollyXOffset() + "</Position X>" + "\n <Position Y>"
            + view3d.getCameraController().getDollyYOffset() + "</Position Y>" + "\n <Position Z>" + view3d.getCameraController().getDollyZOffset()
            + "</Position Z>";
      textToWrite += "\n<Dolly Booleans>" + "\n<Dolly>" + view3d.getCameraController().isDolly() + "</Dolly>" + "\n<Dolly X>"
            + view3d.getCameraController().isDollyX() + "</Dolly X>" + "\n<Dolly Y>" + view3d.getCameraController().isDollyY() + "</Dolly Y>" + "\n<Dolly Z>"
            + view3d.getCameraController().isDollyY() + "</Dolly Z>" + "\n</Dolly Booleans>";
      textToWrite += "\n</Dolly data>";
      textToWrite += "\n <Track data> \n <Position X>" + view3d.getCameraController().getTrackingXOffset() + "</Position X>" + "\n <Position Y>"
            + view3d.getCameraController().getTrackingYOffset() + "</Position Y>" + "\n <Position Z>" + view3d.getCameraController().getTrackingZOffset()
            + "</Position Z>";
      textToWrite += "\n <Track Booleans>" + "\n<Track>" + view3d.getCameraController().isTracking() + "</Track>" + "\n<Track X>"
            + view3d.getCameraController().isTrackingX() + "</Track X>" + "\n<Track Y>" + view3d.getCameraController().isTrackingY() + "</Track Y>"
            + "\n<Track Z>" + view3d.getCameraController().isTracking() + "</Track Z>" + "\n</Track Booleans>";
      textToWrite += "\n</Track data>";
      textToWrite += "\n<Fix Position>" + "\n<Fix X>" + view3d.getCameraController().getFixX() + "</Fix X>" + "\n<Fix Y>"
            + view3d.getCameraController().getFixY() + "</Fix Y>" + "\n<Fix Z>" + view3d.getCameraController().getFixZ() + "</Fix Z>" + "\n</Fix Position>";
      textToWrite += "\n</Canvas" + canvasNumber + ">";

      return textToWrite;
   }

   public boolean setMainViewPortFromXMLDescription(String importXML)
   {
      String textToLoad = XMLReaderUtility.getMiddleString(0, importXML, "<Main Viewport>", "</Main Viewport>");
      String visible = XMLReaderUtility.getMiddleString(0, textToLoad, "<Visible>", "</Visible>");

      return visible.trim().equalsIgnoreCase("true");
   }

   @Override
   public Dimension getMinimumSize()
   {
      return new Dimension(0, 0);
   }

   private int sizeForSetUpOfMultiViews(String currentView)
   {
      int size = 1;

      if (currentView.equals("Split Screen"))
      {
         size = 2;
      }

      if (currentView.equals("Three Views"))
      {
         size = 3;
      }

      if (currentView.equals("Four Views"))
      {
         size = 4;
      }

      return size;
   }

   public void setupMultiViews(String xmlRepresentation, String currentView)
   {
      for (int i = 0; i < sizeForSetUpOfMultiViews(currentView); i++)
      {
         int label = i + 1;
         String first = "<Canvas " + label + ">";
         String second = "</Canvas " + label + ">";
         String textToLoad = XMLReaderUtility.getMiddleString(0, xmlRepresentation, first, second);
         double posX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Camera X>", "</Camera X>"));
         double posY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Camera Y>", "</Camera Y>"));
         double posZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Camera Z>", "</Camera Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setCameraPosition(posX, posY, posZ);

         String Dolly = XMLReaderUtility.getMiddleString(0, textToLoad, "<Dolly data>", "</Dolly data>");
         double DollyX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position X>", "</Position X>"));
         double DollyY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Y>", "</Position Y>"));
         double DollyZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Z>", "</Position Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setDollyOffsets(DollyX, DollyY, DollyZ);

         String Dolly_Boolean = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly>", "</Dolly>");
         String Dolly_Boolean_X = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly X>", "</Dolly X>");
         String Dolly_Boolean_Y = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Y>", "</Dolly Y>");
         String Dolly_Boolean_Z = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Z>", "</Dolly Z>");
         boolean dolly_set = true;
         boolean dolly_setX = true;
         boolean dolly_setY = true;
         boolean dolly_setZ = true;

         if (Dolly_Boolean.equals("false"))
         {
            dolly_set = false;
         }

         if (Dolly_Boolean_X.equals("false"))
         {
            dolly_setX = false;
         }

         if (Dolly_Boolean_Y.equals("false"))
         {
            dolly_setY = false;
         }

         if (Dolly_Boolean_Z.equals("false"))
         {
            dolly_setZ = false;
         }

         canvasPanels.get(i).getStandard3DView().getCameraController().setDolly(dolly_set, dolly_setX, dolly_setY, dolly_setZ);

         String Track = XMLReaderUtility.getMiddleString(0, textToLoad, "<Track data>", "</Track data>");
         double TrackX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position X>", "</Position X>"));
         double TrackY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Y>", "</Position Y>"));
         double TrackZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Z>", "</Position Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setTrackingOffsets(TrackX, TrackY, TrackZ);

         String Track_Boolean = XMLReaderUtility.getMiddleString(0, Track, "<Track>", "</Track>");
         String Track_Boolean_X = XMLReaderUtility.getMiddleString(0, Track, "<Track X>", "</Track X>");
         String Track_Boolean_Y = XMLReaderUtility.getMiddleString(0, Track, "<Track Y>", "</Track Y>");
         String Track_Boolean_Z = XMLReaderUtility.getMiddleString(0, Track, "<Track Z>", "</Track Z>");
         boolean track_set = true;
         boolean track_setX = true;
         boolean track_setY = true;
         boolean track_setZ = true;

         if (Track_Boolean.equals("false"))
         {
            track_set = false;
         }

         if (Track_Boolean_X.equals("false"))
         {
            track_setX = false;
         }

         if (Track_Boolean_Y.equals("false"))
         {
            track_setY = false;
         }

         if (Track_Boolean_Z.equals("false"))
         {
            track_setZ = false;
         }

         canvasPanels.get(i).getStandard3DView().getCameraController().setTracking(track_set, track_setX, track_setY, track_setZ);

         String Fix = XMLReaderUtility.getMiddleString(0, textToLoad, "<Fix Position>", "</Fix Position>");
         double FixX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix X>", "</Fix X>"));
         double FixY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Y>", "</Fix Y>"));
         double FixZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Z>", "</Fix Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setFixPosition(FixX, FixY, FixZ);
      }
   }

   public void setupMultiViews_ViewportWindows(String xmlRepresentation, String currentView)
   {
      for (int i = 0; i < sizeForSetUpOfMultiViews(currentView); i++)
      {
         int label = i + 1;
         String first = "<Canvas" + label + ">";
         String second = "</Canvas" + label + ">";
         String textToLoad = XMLReaderUtility.getMiddleString(0, xmlRepresentation, first, second);
         double posX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera X>", "</Viewport Camera X>"));
         double posY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera Y>", "</Viewport Camera Y>"));
         double posZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera Z>", "</Viewport Camera Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setCameraPosition(posX, posY, posZ);

         String Dolly = XMLReaderUtility.getMiddleString(0, textToLoad, "<Dolly data>", "</Dolly data>");
         double DollyX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position X>", "</Position X>"));
         double DollyY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Y>", "</Position Y>"));
         double DollyZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Z>", "</Position Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setDollyOffsets(DollyX, DollyY, DollyZ);

         String Dolly_Boolean = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly>", "</Dolly>");
         String Dolly_Boolean_X = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly X>", "</Dolly X>");
         String Dolly_Boolean_Y = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Y>", "</Dolly Y>");
         String Dolly_Boolean_Z = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Z>", "</Dolly Z>");
         boolean dolly_set = true;
         boolean dolly_setX = true;
         boolean dolly_setY = true;
         boolean dolly_setZ = true;

         if (Dolly_Boolean.equals("false"))
         {
            dolly_set = false;
         }

         if (Dolly_Boolean_X.equals("false"))
         {
            dolly_setX = false;
         }

         if (Dolly_Boolean_Y.equals("false"))
         {
            dolly_setY = false;
         }

         if (Dolly_Boolean_Z.equals("false"))
         {
            dolly_setZ = false;
         }

         canvasPanels.get(i).getStandard3DView().getCameraController().setDolly(dolly_set, dolly_setX, dolly_setY, dolly_setZ);

         String Track = XMLReaderUtility.getMiddleString(0, textToLoad, "<Track data>", "</Track data>");
         double TrackX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position X>", "</Position X>"));
         double TrackY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Y>", "</Position Y>"));
         double TrackZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Z>", "</Position Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setTrackingOffsets(TrackX, TrackY, TrackZ);

         String Track_Boolean = XMLReaderUtility.getMiddleString(0, Track, "<Track>", "</Track>");
         String Track_Boolean_X = XMLReaderUtility.getMiddleString(0, Track, "<Track X>", "</Track X>");
         String Track_Boolean_Y = XMLReaderUtility.getMiddleString(0, Track, "<Track Y>", "</Track Y>");
         String Track_Boolean_Z = XMLReaderUtility.getMiddleString(0, Track, "<Track Z>", "</Track Z>");
         boolean track_set = true;
         boolean track_setX = true;
         boolean track_setY = true;
         boolean track_setZ = true;

         if (Track_Boolean.equals("false"))
         {
            track_set = false;
         }

         if (Track_Boolean_X.equals("false"))
         {
            track_setX = false;
         }

         if (Track_Boolean_Y.equals("false"))
         {
            track_setY = false;
         }

         if (Track_Boolean_Z.equals("false"))
         {
            track_setZ = false;
         }

         canvasPanels.get(i).getStandard3DView().getCameraController().setTracking(track_set, track_setX, track_setY, track_setZ);

         String Fix = XMLReaderUtility.getMiddleString(0, textToLoad, "<Fix Position>", "</Fix Position>");
         double FixX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix X>", "</Fix X>"));
         double FixY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Y>", "</Fix Y>"));
         double FixZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Z>", "</Fix Z>"));

         canvasPanels.get(i).getStandard3DView().getCameraController().setFixPosition(FixX, FixY, FixZ);
      }
   }

   public void setCameraTracking(boolean cameraTracking)
   {
      // TODO: make viewports based on interface
   }

   private void setCameraConfiguration(TrackingDollyCameraController camera, CameraConfiguration config, YoVariableHolder holder, CameraMountList mountList)
   {
      camera.setConfiguration(config, mountList);

      CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyYoVariablesHolder = (CameraTrackAndDollyYoVariablesHolder) camera
            .getCameraTrackAndDollyVariablesHolder();

      DoubleYoVariable trackXVar = (DoubleYoVariable) holder.getVariable(config.getTrackXVar());
      DoubleYoVariable trackYVar = (DoubleYoVariable) holder.getVariable(config.getTrackYVar());
      DoubleYoVariable trackZVar = (DoubleYoVariable) holder.getVariable(config.getTrackZVar());
      cameraTrackAndDollyYoVariablesHolder.setTrackingVars(trackXVar, trackYVar, trackZVar);

      DoubleYoVariable dollyXVar = (DoubleYoVariable) holder.getVariable(config.getDollyXVar());
      DoubleYoVariable dollyYVar = (DoubleYoVariable) holder.getVariable(config.getDollyYVar());
      DoubleYoVariable dollyZVar = (DoubleYoVariable) holder.getVariable(config.getDollyZVar());
      cameraTrackAndDollyYoVariablesHolder.setDollyVars(dollyXVar, dollyYVar, dollyZVar);

      if (config.getFieldOfViewVar() != null)
      {
         DoubleYoVariable fieldOfViewVar = (DoubleYoVariable) yoVariableHolder.getVariable(config.getFieldOfViewVar());
         cameraTrackAndDollyYoVariablesHolder.setFieldOfViewVar(fieldOfViewVar);
      }

      if (standardGUIActions != null)
      {
         standardGUIActions.makeCheckBoxesConsistentWithCamera();
      }
   }


}
