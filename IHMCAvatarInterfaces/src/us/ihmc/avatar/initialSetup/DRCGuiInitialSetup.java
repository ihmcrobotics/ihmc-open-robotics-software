package us.ihmc.avatar.initialSetup;

import javax.swing.JButton;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.visualization.WalkControllerSliderBoard;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.dataExporter.TorqueSpeedDataExporter;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class DRCGuiInitialSetup implements GuiInitialSetup
{
   private static final boolean SHOW_ONLY_WRENCH_VISUALIZER = false;
   private static final boolean SHOW_EXPORT_TORQUE_AND_SPEED = true;
   private static final boolean SHOW_OVERHEAD_VIEW = true;
   public static final boolean MAKE_SLIDER_BOARD = false;

   private final SimulationConstructionSetParameters simulationConstructionSetParameters;


   private boolean is3dGraphicsShown = true;
   private final boolean groundProfileVisible;
   private final boolean drawPlaneAtZ0;
   private boolean showOverheadView = SHOW_OVERHEAD_VIEW;

   public DRCGuiInitialSetup(boolean groundProfileVisible, boolean drawPlaneAtZeroHeight, SimulationConstructionSetParameters simulationConstructionSetParameters)
   {
      this.groundProfileVisible = groundProfileVisible;
      this.drawPlaneAtZ0 = drawPlaneAtZeroHeight;
      this.simulationConstructionSetParameters = simulationConstructionSetParameters;
   }

   public DRCGuiInitialSetup(boolean groundProfileVisible, boolean drawPlaneAtZeroHeight)
   {
      this(groundProfileVisible, drawPlaneAtZeroHeight, true);
   }

   public DRCGuiInitialSetup(boolean groundProfileVisible, boolean drawPlaneAtZeroHeight, boolean showGUI)
   {
      this(groundProfileVisible, drawPlaneAtZeroHeight, new SimulationConstructionSetParameters(showGUI));
   }

   public SimulationConstructionSetParameters getSimulationConstructionSetParameters()
   {
      return simulationConstructionSetParameters;
   }

   public void initializeGUI(SimulationConstructionSet scs, Robot robot)
   {
      // use
      // public void initializeGUI(SimulationConstructionSet scs, Robot robot, DRCRobotModel robotModel)
      throw new RuntimeException("Should not be here. This function is a relict of the GuiInitialSetup interface.");
   }

   public void initializeGUI(SimulationConstructionSet scs, Robot robot, DRCRobotModel robotModel)
   {
      CameraConfiguration behindPelvis = new CameraConfiguration("BehindPelvis");
      behindPelvis.setCameraTracking(false, true, true, false);
      behindPelvis.setCameraDolly(false, true, true, false);
      behindPelvis.setCameraFix(0.0, 0.0, 1.0);
      behindPelvis.setCameraPosition(-2.5, 0.0, 1.0);
      behindPelvis.setCameraTrackingVars("q_x", "q_y", "q_z");
      scs.setupCamera(behindPelvis);

      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotCameraParameters[] cameraInfo = sensorInformation.getCameraParameters();
      if (cameraInfo != null)
      {
         for (int i = 0; i < cameraInfo.length; i++)
         {
            CameraConfiguration camera = new CameraConfiguration(cameraInfo[i].getSensorNameInSdf());
            camera.setCameraMount(cameraInfo[i].getSensorNameInSdf());
            scs.setupCamera(camera);
         }
      }

      scs.setGroundVisible(groundProfileVisible);

      if (drawPlaneAtZ0)
      {
         Graphics3DObject planeAtZ0 = new Graphics3DObject();

         HeightMap heightMap = new FlatGroundProfile();
         planeAtZ0.addHeightMap(heightMap, 100, 100, null);
         scs.addStaticLinkGraphics(planeAtZ0);
      }

      if (!is3dGraphicsShown)
      {
         scs.hideViewport();
      }

      if (SHOW_ONLY_WRENCH_VISUALIZER)
      {
         scs.hideAllDynamicGraphicObjects();
         scs.setDynamicGraphicObjectsListVisible("wrenchVisualizer", true);
      }

      if (SHOW_EXPORT_TORQUE_AND_SPEED)
      {
         JButton exportTorqueAndSpeedButton = new JButton("Export Torque And Speed");
         TorqueSpeedDataExporter dataExporter = new TorqueSpeedDataExporter(scs, robot);
         exportTorqueAndSpeedButton.addActionListener(dataExporter);
         scs.addButton(exportTorqueAndSpeedButton);
      }

      //TODO: Clean this up!
//      GeneralizedSDFRobotModel generalizedSDFRobotModel = robotModel.getGeneralizedRobotModel();

      if (MAKE_SLIDER_BOARD)
      {
         new WalkControllerSliderBoard(scs, scs.getRootRegistry(), null);
      }
   }

   public boolean isGuiShown()
   {
      return simulationConstructionSetParameters.getCreateGUI();
   }

   public boolean getShowWindow()
   {
      return simulationConstructionSetParameters.getShowWindows();
   }

   public void setCreateGUI(boolean createGUI)
   {
      this.simulationConstructionSetParameters.setCreateGUI(createGUI);
   }

   public void setShowWindow(boolean showWindow)
   {
      this.simulationConstructionSetParameters.setShowSplashScreen(showWindow);
      this.simulationConstructionSetParameters.setShowWindows(showWindow);

   }

   public Graphics3DAdapter getGraphics3DAdapter()
   {
      if (simulationConstructionSetParameters.getCreateGUI() && is3dGraphicsShown)
      {
         return new JMEGraphics3DAdapter();
      }
      else if (simulationConstructionSetParameters.getCreateGUI())
      {
         return new NullGraphics3DAdapter();
      }
      else
      {
         return null;
      }
   }

   public void setIs3dGraphicsShown(boolean is3dGraphicsShown)
   {
      this.is3dGraphicsShown = is3dGraphicsShown;
   }

   public boolean isShowOverheadView()
   {
      return showOverheadView;
   }

   public void setShowOverheadView(boolean showOverheadView)
   {
      this.showOverheadView = showOverheadView;
   }
}
