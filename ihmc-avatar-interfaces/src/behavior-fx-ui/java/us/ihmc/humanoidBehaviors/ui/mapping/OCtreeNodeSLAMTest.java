package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;

public class OCtreeNodeSLAMTest extends Application
{
   private final List<StereoVisionPointCloudMessage> messagesFromFile = new ArrayList<>();

   // 25 26 big translation (?)   25 28 big z translation. 27 28 small z translation
   private final int indexFrameOne = 25;
   private final int indexFrameTwo = 28;

   private final String stereoPath = "E:\\Data\\Walking10\\PointCloud\\";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messagesFromFile.addAll(StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath)));
      System.out.println("number of messages " + messagesFromFile.size());
      
      OctreeNodeSLAM slam = new OctreeNodeSLAM(messagesFromFile.get(indexFrameOne));
      slam.addFrame(messagesFromFile.get(indexFrameTwo));

      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      StereoVisionPointCloudGraphic stereoVisionPointCloudGraphic = new StereoVisionPointCloudGraphic();
      NormalOctreeGraphic octreeGraphic = new NormalOctreeGraphic();

      stereoVisionPointCloudGraphic.initializeMeshes();
      stereoVisionPointCloudGraphic.addSingleMesh(slam.getRawPointCloudMap().get(0), Color.BLACK);
      stereoVisionPointCloudGraphic.addSingleMesh(slam.getRawPointCloudMap().get(1), Color.BLUE);
      
      stereoVisionPointCloudGraphic.addPointsMeshes(slam.gePointCloudMap().get(0), Color.RED);
      stereoVisionPointCloudGraphic.addPointsMeshes(slam.gePointCloudMap().get(1), Color.ORANGE);
      
      stereoVisionPointCloudGraphic.generateMeshes();
      stereoVisionPointCloudGraphic.update();
      view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);

      regionsGraphic.generateMeshes(slam.getPlanarRegionsList());
      regionsGraphic.update();
      view3dFactory.addNodeToView(regionsGraphic);
      
      octreeGraphic.initialize();
//      octreeGraphic.addMesh(slam.getOctreeMap().get(0), OctreeNodeSLAM.OCTREE_RESOLUTION, Color.BEIGE);
//      octreeGraphic.addMesh(slam.planes, OctreeNodeSLAM.OCTREE_RESOLUTION, Color.GREEN);
      octreeGraphic.generateMeshes();
      octreeGraphic.update();
      view3dFactory.addNodeToView(octreeGraphic);

      primaryStage.setTitle(this.getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
