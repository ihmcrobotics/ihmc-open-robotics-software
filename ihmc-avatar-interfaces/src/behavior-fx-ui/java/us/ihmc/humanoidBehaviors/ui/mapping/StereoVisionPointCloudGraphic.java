package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;

public class StereoVisionPointCloudGraphic extends Group
{
   private static final float ORIGIN_POINT_SIZE = 0.05f;
   private static final float SCAN_POINT_SIZE = 0.0075f;
   private static final int NUMBER_OF_POINTS_PER_MESSAGE = 2000;

   private static final Color DEFAULT_COLOR = Color.BLACK;

   private static final int palleteSizeForMeshBuilder = 2048;

   private volatile List<Node> messageNodes;
   private List<Node> lastNodes = null;
   private volatile List<Node> updatePointCloudMeshViews;

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   public StereoVisionPointCloudGraphic()
   {
      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(palleteSizeForMeshBuilder));
   }

   private void addMesh(StereoVisionPointCloudMessage stereoVisionPointCloudMessage, Color colorToViz)
   {
      Point3D32 sensorPosition = new Point3D32();
      Point3D32 scanPoint = new Point3D32();
      sensorPosition.set(stereoVisionPointCloudMessage.getSensorPosition());
      int redScaler = (int) (0xFF * (1 - (stereoVisionPointCloudMessage.getSensorPoseConfidence())));
      int greenScaler = (int) (0xFF * (stereoVisionPointCloudMessage.getSensorPoseConfidence()));
      Color confidenceColor = Color.rgb(redScaler, greenScaler, 0);
      meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(ORIGIN_POINT_SIZE), sensorPosition, confidenceColor);

      int numberOfScanPoints = stereoVisionPointCloudMessage.getPointCloud().size() / 3;
      int sizeOfPointCloudToVisualize = Math.min(numberOfScanPoints, NUMBER_OF_POINTS_PER_MESSAGE);
      Random random = new Random();
      for (int j = 0; j < sizeOfPointCloudToVisualize; j++)
      {
         int indexToVisualize;
         if (numberOfScanPoints < NUMBER_OF_POINTS_PER_MESSAGE)
            indexToVisualize = j;
         else
            indexToVisualize = random.nextInt(numberOfScanPoints);

         MessageTools.unpackScanPoint(stereoVisionPointCloudMessage, indexToVisualize, scanPoint);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, colorToViz);
      }
   }

   /**
    * for test
    * ===============================================================================
    */
   // TODO : remove.
   public void addPointsMeshes(Point3DReadOnly[] points, Color colorToViz)
   {
      Point3D32 scanPoint = new Point3D32();

      int numberOfScanPoints = points.length;
      int sizeOfPointCloudToVisualize = Math.min(numberOfScanPoints, NUMBER_OF_POINTS_PER_MESSAGE);
      for (int j = 0; j < sizeOfPointCloudToVisualize; j++)
      {
         scanPoint.set(points[j]);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, colorToViz);
      }
   }

   public void addPointsMeshes(Point3DReadOnly[] points, RigidBodyTransformReadOnly sensorPose, Color colorToViz)
   {
      Point3D32 sensorPosition = new Point3D32();
      sensorPosition.set(sensorPose.getTranslation());
      Color color = Color.rgb(0, 0xFF, 0);
      meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(ORIGIN_POINT_SIZE), sensorPosition, color);
      addPointsMeshes(points, colorToViz);
   }

   public void addSingleMesh(StereoVisionPointCloudMessage stereoVisionPointCloudMessage, Color colorToViz)
   {
      addMesh(stereoVisionPointCloudMessage, colorToViz);
   }

   public void initializeMeshes()
   {
      updatePointCloudMeshViews = new ArrayList<>();
      meshBuilder.clear();
   }

   public void generateMeshes()
   {
      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());

      updatePointCloudMeshViews.add(scanMeshView);
      meshBuilder.clear();
      messageNodes = updatePointCloudMeshViews;
   }

   /**
    * for test
    * ===============================================================================
    */

   public void generateMeshes(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      updatePointCloudMeshViews = new ArrayList<>();

      meshBuilder.clear();

      addMesh(stereoVisionPointCloudMessage, DEFAULT_COLOR);

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());

      updatePointCloudMeshViews.add(scanMeshView);

      meshBuilder.clear();
      messageNodes = updatePointCloudMeshViews;
   }

   public void generateMeshes(List<StereoVisionPointCloudMessage> stereoVisionPointCloudMessages)
   {
      updatePointCloudMeshViews = new ArrayList<>();

      meshBuilder.clear();
      for (int i = 0; i < stereoVisionPointCloudMessages.size(); i++)
         addMesh(stereoVisionPointCloudMessages.get(i), DEFAULT_COLOR);

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());

      updatePointCloudMeshViews.add(scanMeshView);

      meshBuilder.clear();
      messageNodes = updatePointCloudMeshViews;
   }

   public void update()
   {
      List<Node> meshViews = messageNodes;
      if (lastNodes != meshViews)
      {
         getChildren().clear();
         getChildren().addAll(meshViews);
         lastNodes = meshViews;
      }
   }
}
