package us.ihmc.avatar.heightMap;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import tf2_msgs.TFMessage;
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
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.net.URI;
import java.util.concurrent.atomic.AtomicReference;

public abstract class HeightMapUI extends ApplicationNoModule
{
   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(HeightMapMessagerAPI.API);
   private static final boolean useROS2 = false;

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
      HeightMapParameters parameters = new HeightMapParameters();

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "height_map");
      new HeightMapUpdater(messager, ros2Node, stage);

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(HeightMapUI.class.getResource("HeightMapUI.fxml"));
      mainPane = loader.load();
      SplitPane splitPane = (SplitPane) mainPane.getCenter();
      BorderPane centerBorderPane = (BorderPane) splitPane.getItems().get(0);

      DRCRobotModel robotModel = getRobotModel();
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      URI rosuri = NetworkParameters.getROSURI();
//      URI rosuri = new URI("http://172.16.66.102:11311");

      ros1Node = RosTools.createRosNode(rosuri, "height_map_viewer");
      AtomicReference<Transform> ros1OusterTransform = new AtomicReference<>();

      ros1Node.attachSubscriber("os_cloud_node2/points", new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            FramePose3D ousterPose = new FramePose3D();
            if (useROS2)
            {
               syncedRobot.update();
               ousterPose.setToZero(syncedRobot.getReferenceFrames().getOusterLidarFrame());
               ousterPose.changeFrame(ReferenceFrame.getWorldFrame());
            }
            else if (ros1OusterTransform.get() != null)
            {
               Transform ousterTransform = ros1OusterTransform.get();
               Vector3 ros1Translation = ousterTransform.getTranslation();
               Quaternion ros1Orientation = ousterTransform.getRotation();

               ousterPose.getPosition().set(ros1Translation.getX(), ros1Translation.getY(), ros1Translation.getZ());
               ousterPose.getOrientation().set(ros1Orientation.getX(), ros1Orientation.getY(), ros1Orientation.getZ(), ros1Orientation.getW());
            }

            messager.submitMessage(HeightMapMessagerAPI.PointCloudData, Pair.of(pointCloud, ousterPose));
         }
      });

      if (!useROS2)
      {
         ros1Node.attachSubscriber("/tf", new AbstractRosTopicSubscriber<TFMessage>(TFMessage._TYPE)
         {
            @Override
            public void onNewMessage(TFMessage message)
            {
               for (int i = 0; i < message.getTransforms().size(); i++)
               {
                  TransformStamped transform = message.getTransforms().get(i);
                  if (transform.getChildFrameId().equals("os_sensor"))
                  {
                     ros1OusterTransform.set(transform.getTransform());
                  }
               }
            }
         });
      }

      stage.setTitle(getClass().getSimpleName());

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      if (SHOW_HEIGHT_MAP)
      {
         heightMapVisualizer = new HeightMapVisualizer();
         messager.registerTopicListener(HeightMapMessagerAPI.HeightMapData, heightMapVisualizer::update);
         messager.registerTopicListener(HeightMapMessagerAPI.MaxHeight, heightMapVisualizer::setMaxHeight);
         messager.registerTopicListener(HeightMapMessagerAPI.xPosition, v -> heightMapVisualizer.setDebugPosition(0, v));
         messager.registerTopicListener(HeightMapMessagerAPI.yPosition, v -> heightMapVisualizer.setDebugPosition(1, v));
         messager.registerTopicListener(HeightMapMessagerAPI.zPosition, v -> heightMapVisualizer.setDebugPosition(2, v));
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
      heightMapParametersUIController.attachMessager(messager);
      heightMapParametersUIController.bindControls();

      view3dFactory.bindSubSceneSizeToPaneSize(mainPane);
      centerBorderPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      stage.setScene(new Scene(mainPane, 1200, 800, true));
      stage.show();

      ros1Node.execute();
      messager.startMessager();

      stage.setOnCloseRequest(event -> stop());
      heightMapParametersUIController.onPrimaryStageLoaded();

      int initialPublishFrequency = 5;
      messager.submitMessage(HeightMapMessagerAPI.PublishFrequency, initialPublishFrequency);
      messager.submitMessage(HeightMapMessagerAPI.EnableUpdates, true);
      messager.submitMessage(HeightMapMessagerAPI.GridCenterX, 2.0);
      messager.submitMessage(HeightMapMessagerAPI.GridCenterY, 0.0);
   }

   public void stop()
   {
      if (SHOW_HEIGHT_MAP)
         heightMapVisualizer.stop();
      if (SHOW_POINT_CLOUD)
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
