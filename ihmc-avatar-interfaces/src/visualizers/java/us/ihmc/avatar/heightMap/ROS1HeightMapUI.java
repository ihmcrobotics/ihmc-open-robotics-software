package us.ihmc.avatar.heightMap;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class ROS1HeightMapUI extends Application
{
   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(HeightMapMessagerAPI.API);

   private HeightMapVisualizer heightMapVisualizer;
   private final PointCloudVisualizer pointCloudVisualizer = new PointCloudVisualizer();

   private RosMainNode ros1Node;

   private static final boolean SHOW_HEIGHT_MAP = true;
   private static final boolean SHOW_POINT_CLOUD = false;

   @Override
   public void start(Stage stage) throws Exception
   {
      new HeightMapUpdater(messager);
      heightMapVisualizer = new HeightMapVisualizer(messager);

      ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "height_map_viewer");
      ros1Node.attachSubscriber(RosTools.OUSTER_POINT_CLOUD, new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            messager.submitMessage(HeightMapMessagerAPI.PointCloudData, pointCloud);
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

      heightMapVisualizer.start();
      pointCloudVisualizer.start();

      BorderPane mainPane = new BorderPane();
      view3dFactory.bindSubSceneSizeToPaneSize(mainPane);
      mainPane.setCenter(view3dFactory.getSubScene());

      stage.setScene(new Scene(mainPane, 1200, 800, true));
      stage.show();

      ros1Node.execute();
      messager.startMessager();

      stage.setOnCloseRequest(event -> stop());
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

   public static void main(String[] args)
   {
      Application.launch();
   }
}
