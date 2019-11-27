package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class InverseExtrapolationTestViewer extends Application
{
   private final List<StereoVisionPointCloudMessage> messagesFromFile = new ArrayList<>();

   // 25 26 big translation   24 25 has small translation   25 28 big z translation. 27 28 small z translation
   private final int indexFrameOne = 45;
   private final int indexFrameTwo = 46;

   //private final String stereoPath = "E:\\Data\\Walking10\\PointCloud\\";
   private final String stereoPath = "E:\\Data\\Walking7-fixedframe\\PointCloud\\";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      /**
       * "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\"
       * 1574281446221091367 and 1574281446745168115 has discontinuity.
       */
      //      numbersToViz.add("1574281446221091367");   16 
      //      numbersToViz.add("1574281446745168115");   17

      /**
       * "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\"
       * 1574281444731577987 and 1574281445344718719 has small overlap area but merge-able.
       */
      //            numbersToViz.add("1574281444731577987");   14
      //            numbersToViz.add("1574281445344718719");   15
      //      numbersToViz.add("1574281446745168115");   18

      /**
       * "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\"
       * 1574281443383280234 and 1574281443799487886 has big overlap area and merge-able.
       */
      //            numbersToViz.add("1574281443383280234");   10
      //            numbersToViz.add("1574281443799487886");   11

      messagesFromFile.addAll(StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath)));
      System.out.println("number of messages " + messagesFromFile.size());

      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      StereoVisionPointCloudGraphic stereoVisionPointCloudGraphic = new StereoVisionPointCloudGraphic();

      Point3D[] points = getPointsFromMessage(messagesFromFile.get(indexFrameOne));
      Point3D[] pointsPrevious = getPointsFromMessage(messagesFromFile.get(indexFrameTwo));
      RigidBodyTransform sensorPose = getSensorPoseFromMessage(messagesFromFile.get(indexFrameOne));
      RigidBodyTransform previousSensorPose = getSensorPoseFromMessage(messagesFromFile.get(indexFrameTwo));
      Point3D[] pointsInPreviousView = EnvironmentMappingTools.createPointsInPreviousView(sensorPose, previousSensorPose, points);
      System.out.println("pointsInPreviousView " + pointsInPreviousView.length);

      stereoVisionPointCloudGraphic.initializeMeshes();
      stereoVisionPointCloudGraphic.addPointsMeshes(points, Color.BLUE);
      stereoVisionPointCloudGraphic.addPointsMeshes(pointsPrevious, Color.RED);
      //stereoVisionPointCloudGraphic.addPointsMeshes(pointsInPreviousView, Color.GREEN);
      stereoVisionPointCloudGraphic.generateMeshes();

      stereoVisionPointCloudGraphic.update();
      view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);

      InverseExtrapolationStereoFrameFilter frameFilter = new InverseExtrapolationStereoFrameFilter();
      EnvironmentMap environmentMap = new EnvironmentMap(0, messagesFromFile.get(indexFrameOne));
      boolean result = frameFilter.intersectionFilter(messagesFromFile.get(indexFrameTwo), environmentMap);
      System.out.println("result " + result);

      environmentMap.addFrame(0, messagesFromFile.get(indexFrameTwo));

      regionsGraphic.generateMeshes(environmentMap.getPlanarRegionsMap());
      regionsGraphic.update();
      //      view3dFactory.addNodeToView(regionsGraphic);

      primaryStage.setTitle(this.getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public Point3D[] getPointsFromMessage(StereoVisionPointCloudMessage message)
   {
      int numberOfPoints = message.getColors().size();
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = new Point3D();
         MessageTools.unpackScanPoint(message, i, pointCloud[i]);
      }
      return pointCloud;
   }

   public RigidBodyTransform getSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
