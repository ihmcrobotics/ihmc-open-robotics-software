package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.Gdx;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.nadia.parameters.NadiaRobotModel;
import us.ihmc.nadia.parameters.robotVersions.NadiaVersion;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Activator;

public class GDXManualPointCloudMapViewer
{
   private final Activator nativesLoadedActivator;
   private final CommunicationMode ros2CommunicationMode;
   private final GDXImGuiBasedUI baseUI;
   private final ROS2SyncedRobotModel syncedRobot;

   private final ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;
   private final GDXEnvironmentBuilder environmentBuilder;
   private int renderNumber = 0;


   public GDXManualPointCloudMapViewer(DRCRobotModel robotModel)
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
      ros2CommunicationMode = CommunicationMode.INTERPROCESS;

//      NadiaRobotModel robotModel = new NadiaRobotModel(NadiaVersion.V16_NEW_TORSO_FULL_ROBOT, RobotTarget.REAL_ROBOT);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(ros2CommunicationMode.getPubSubImplementation(), "simulation_ui");
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      String repoName = "nadia";
      String subsequentPathToResourceFolder = "nadia-user-interface/src/libgdx/resources";
      baseUI = new GDXImGuiBasedUI(getClass(), repoName, subsequentPathToResourceFolder, "Nadia Manual Map Viewer");

      globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();
      baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);

      environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
      environmentBuilder.getInputsEnabled().set(false);
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(3.0, 1.0, 2.5);

            environmentBuilder.create();

            globalVisualizersUI.addVisualizer(new GDXROS2PointCloudMapVisualizer("Fused point cloud message", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN, syncedRobot));

            globalVisualizersUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.MODEL);
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            syncedRobot.update();

            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               globalVisualizersUI.update();
            }

            if (renderNumber < 10)
            {
               ++renderNumber;
               if (renderNumber == 10)
               {
                  environmentBuilder.loadEnvironment("DemoPullDoor.json");
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            globalVisualizersUI.destroy();
            ros2Node.destroy();
            baseUI.dispose();
         }
      });
   }

   public void exit()
   {
      Gdx.app.exit();
   }

   public static void main(String[] args)
   {
      new GDXManualPointCloudMapViewer(new NadiaRobotModel(NadiaVersion.V16_NEW_TORSO_FULL_ROBOT, RobotTarget.REAL_ROBOT));
   }

}
