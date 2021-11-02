package us.ihmc.avatar.heightMap;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.ui.viewers.HeightMapVisualizer;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public abstract class HeightMapUI extends Application
{
   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(HeightMapMessagerAPI.API);

   private HeightMapVisualizer heightMapVisualizer;
   private PointCloudVisualizer pointCloudVisualizer;

   private ROS2SyncedRobotModel syncedRobot;
   private RosMainNode ros1Node;
   private ROS2Node ros2Node;
   private BorderPane mainPane;

   @FXML
   private HeightMapParametersUIController heightMapParametersUIController;

   private static final boolean SHOW_HEIGHT_MAP = true;
   private static final boolean SHOW_POINT_CLOUD = false;

   @Override
   public void start(Stage stage) throws Exception
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "height_map");
      pointCloudVisualizer = new PointCloudVisualizer(messager);
      new HeightMapUpdater(messager, ros2Node);

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(HeightMapUI.class.getResource("HeightMapUI.fxml"));
      mainPane = loader.load();
      SplitPane splitPane = (SplitPane) mainPane.getCenter();
      BorderPane centerBorderPane = (BorderPane) splitPane.getItems().get(0);

      heightMapVisualizer = new HeightMapVisualizer();
      messager.registerTopicListener(HeightMapMessagerAPI.HeightMapData, heightMapVisualizer::update);

      DRCRobotModel robotModel = getRobotModel();
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "height_map_viewer");
      ros1Node.attachSubscriber(RosTools.OUSTER_POINT_CLOUD, new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            syncedRobot.update();
            FramePose3D ousterPose = new FramePose3D();
            ousterPose.setToZero(syncedRobot.getReferenceFrames().getOusterLidarFrame());
            ousterPose.changeFrame(ReferenceFrame.getWorldFrame());

            messager.submitMessage(HeightMapMessagerAPI.PointCloudData, Pair.of(pointCloud, ousterPose));
         }
      });

      stage.setTitle(getClass().getSimpleName());

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      if (SHOW_HEIGHT_MAP)
         view3dFactory.addNodeToView(heightMapVisualizer.getRoot());
      if (SHOW_POINT_CLOUD)
         view3dFactory.addNodeToView(pointCloudVisualizer.getRoot());

      HeightMapParameters parameters = new HeightMapParameters();
      heightMapParametersUIController.setParameters(parameters);
      heightMapParametersUIController.attachMessager(messager);
      heightMapParametersUIController.bindControls();

      heightMapVisualizer.start();
      pointCloudVisualizer.start();

      view3dFactory.bindSubSceneSizeToPaneSize(mainPane);
//      mainPane.setCenter(view3dFactory.getSubScene());
      centerBorderPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      stage.setScene(new Scene(mainPane, 1200, 800, true));
      stage.show();

      ros1Node.execute();
      messager.startMessager();

      stage.setOnCloseRequest(event -> stop());
      heightMapParametersUIController.onPrimaryStageLoaded();
   }

   public void stop()
   {
      heightMapVisualizer.stop();
      pointCloudVisualizer.stop();
      ros1Node.shutdown();

      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   protected abstract DRCRobotModel getRobotModel();

   public static void main(String[] args)
   {
      Application.launch();
   }
}
