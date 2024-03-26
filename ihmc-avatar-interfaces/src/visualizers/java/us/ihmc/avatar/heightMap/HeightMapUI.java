package us.ihmc.avatar.heightMap;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.ui.viewers.HeightMapVisualizer;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.perception.heightMap.HeightMapInputData;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.perception.gpuHeightMap.HeightMapKernel;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public abstract class HeightMapUI extends ApplicationNoModule
{
   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(HeightMapMessagerAPI.API);

   private HeightMapVisualizer heightMapVisualizer;
   private PointCloudVisualizer pointCloudVisualizer;

   private ROS2SyncedRobotModel syncedRobot;
   private RealtimeROS2Node ros2Node;
   private BorderPane mainPane;
   private final HeightMapKernel heightMapKernel;

   @FXML
   private HeightMapParametersUIController heightMapParametersUIController;

   private static final boolean SHOW_HEIGHT_MAP = true;
   private static final boolean SHOW_POINT_CLOUD = false;

   public HeightMapUI()
   {
      heightMapKernel = new HeightMapKernel();
   }

   public abstract DRCRobotModel getRobotModel();

   @Override
   public void start(Stage stage) throws Exception
   {
      HeightMapParameters parameters = new HeightMapParameters();
      HeightMapFilterParameters filterParameters = new HeightMapFilterParameters();

      ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "height_map");
      new HeightMapUpdaterForUI(messager, ros2Node, stage);


      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(HeightMapUI.class.getResource("HeightMapUI.fxml"));
      mainPane = loader.load();
      SplitPane splitPane = (SplitPane) mainPane.getCenter();
      BorderPane centerBorderPane = (BorderPane) splitPane.getItems().get(0);

      DRCRobotModel robotModel = getRobotModel();
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      ROS2Tools.createCallbackSubscription(ros2Node, PerceptionAPI.OUSTER_LIDAR_SCAN, new NewMessageListener<LidarScanMessage>()
      {
         @Override
         public void onNewDataMessage(Subscriber<LidarScanMessage> subscriber)
         {
            syncedRobot.update();

            double groundHeight = syncedRobot.getReferenceFrames().getMidFeetZUpFrame().getTransformToRoot().getTranslationZ();

            LidarScanMessage data = subscriber.readNextData();

            PointCloudData pointCloudData = new PointCloudData(data);
            Point3D gridCenter = new Point3D(data.getLidarPosition().getX(), data.getLidarPosition().getY(), groundHeight);
            FramePose3D sensorPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());
            HeightMapInputData inputData = new HeightMapInputData();
            inputData.pointCloud = pointCloudData;
            inputData.sensorPose = sensorPose;
            inputData.gridCenter = gridCenter;
            inputData.verticalMeasurementVariance = MathTools.square(parameters.getNominalStandardDeviation());

            messager.submitMessage(HeightMapMessagerAPI.PointCloudData, inputData);
         }
      } );
      /*
      ROS2Tools.createCallbackSubscription(ros2Node, PerceptionAPI.OUSTER_DEPTH_IMAGE, ROS2QosProfile.BEST_EFFORT(), new NewMessageListener<ImageMessage>()
      {
         @Override
         public void onNewDataMessage(Subscriber<ImageMessage> subscriber)
         {
            LogTools.info("Received depth image");
            syncedRobot.update();

            double groundHeight = syncedRobot.getReferenceFrames().getMidFeetZUpFrame().getTransformToRoot().getTranslationZ();

            ImageMessage data = subscriber.readNextData();
            //            FramePose3D ousterPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());
            Point3D gridCenter = new Point3D(data.getPosition().getX(), data.getPosition().getY(), groundHeight);
            PointCloudData pointCloudData = new PointCloudData(perceptionMessageTools, data);
            messager.submitMessage(HeightMapMessagerAPI.PointCloudData, Triple.of(pointCloudData, new FramePose3D(), gridCenter));
         }
      });

       */


      stage.setTitle(getClass().getSimpleName());

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      if (SHOW_HEIGHT_MAP)
      {
         heightMapVisualizer = new HeightMapVisualizer();
         messager.addTopicListener(HeightMapMessagerAPI.HeightMapData, heightMapVisualizer::update);
         messager.addTopicListener(HeightMapMessagerAPI.MaxHeight, heightMapVisualizer::setMaxHeight);
         messager.addTopicListener(HeightMapMessagerAPI.xPosition, v -> heightMapVisualizer.setDebugPosition(0, v));
         messager.addTopicListener(HeightMapMessagerAPI.yPosition, v -> heightMapVisualizer.setDebugPosition(1, v));
         messager.addTopicListener(HeightMapMessagerAPI.zPosition, v -> heightMapVisualizer.setDebugPosition(2, v));
         view3dFactory.addNodeToView(heightMapVisualizer.getRoot());
         heightMapVisualizer.start();
      }
      else
      {
         heightMapVisualizer = null;
      }

      if (SHOW_POINT_CLOUD)
      {
         pointCloudVisualizer = new PointCloudVisualizer(messager, parameters);
         view3dFactory.addNodeToView(pointCloudVisualizer.getRoot());
         pointCloudVisualizer.start();
      }
      else
      {
         pointCloudVisualizer = null;
      }

      heightMapParametersUIController.setParameters(parameters);
      heightMapParametersUIController.setFilterParameters(filterParameters);
      heightMapParametersUIController.attachMessager(messager);
      heightMapParametersUIController.bindControls();

      view3dFactory.bindSubSceneSizeToPaneSize(mainPane);
      centerBorderPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      stage.setScene(new Scene(mainPane, 1200, 800, true));
      stage.show();

      ros2Node.spin();
      messager.startMessager();

      stage.setOnCloseRequest(event -> stop());
//      heightMapParametersUIController.onPrimaryStageLoaded();

      int initialPublishFrequency = 5;
      messager.submitMessage(HeightMapMessagerAPI.PublishFrequency, initialPublishFrequency);
      messager.submitMessage(HeightMapMessagerAPI.EnableUpdates, true);
      messager.submitMessage(HeightMapMessagerAPI.GridCenterX, 2.0);
      messager.submitMessage(HeightMapMessagerAPI.GridCenterY, 0.0);
   }

   public void stop()
   {
      heightMapKernel.destroy();
      ros2Node.destroy();
      if (SHOW_HEIGHT_MAP)
         heightMapVisualizer.stop();
      if (SHOW_POINT_CLOUD)
         pointCloudVisualizer.stop();

      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
