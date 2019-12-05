package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Platform;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMViewer
{
   private static final Color DEFAULT_SENSOR_POSE_COLOR = Color.GREEN;
   private static final Color DEFAULT_POINT_CLOUD_COLOR = Color.BLUE;
   private static final Color DEFAULT_OCTREE_COLOR = Color.BEIGE;

   private final List<PlanarRegionsGraphic> planarRegionsGraphics = new ArrayList<>();
   private final PointCloudGraphic stereoVisionPointCloudGraphic = new PointCloudGraphic();
   private final NormalOctreeGraphic normalOctreeGraphic = new NormalOctreeGraphic();

   public IhmcSLAMViewer()
   {
      stereoVisionPointCloudGraphic.initializeMeshes();
      normalOctreeGraphic.initialize();
   }

   public void addSensorPose(RigidBodyTransformReadOnly sensorPose)
   {
      addSensorPose(sensorPose, DEFAULT_SENSOR_POSE_COLOR);
   }

   public void addPointCloud(Point3DReadOnly[] pointCloud)
   {
      addPointCloud(pointCloud, DEFAULT_POINT_CLOUD_COLOR);
   }

   public void addStereoMessage(StereoVisionPointCloudMessage message)
   {
      addStereoMessage(message, DEFAULT_SENSOR_POSE_COLOR, DEFAULT_POINT_CLOUD_COLOR);
   }

   public void addOctree(NormalOcTree octree, double octreeResolution)
   {
      addOctree(octree, DEFAULT_OCTREE_COLOR, octreeResolution);
   }

   public void addSensorPose(RigidBodyTransformReadOnly sensorPose, Color color)
   {
      stereoVisionPointCloudGraphic.addSensorPoseMesh(sensorPose, color);
   }

   public void addPointCloud(Point3DReadOnly[] point3dReadOnlies, Color color)
   {
      stereoVisionPointCloudGraphic.addPointsMeshes(point3dReadOnlies, color);
   }

   public void addStereoMessage(StereoVisionPointCloudMessage message, Color pointCloudColor)
   {
      stereoVisionPointCloudGraphic.addStereoVisionPointCloudMessageMesh(message, pointCloudColor);
   }

   public void addStereoMessage(StereoVisionPointCloudMessage message, Color sensorPoseColor, Color pointCloudColor)
   {
      stereoVisionPointCloudGraphic.addPointsMeshes(IhmcSLAMTools.extractPointsFromMessage(message), pointCloudColor);
      stereoVisionPointCloudGraphic.addSensorPoseMesh(IhmcSLAMTools.extractSensorPoseFromMessage(message), sensorPoseColor);
   }

   public void addOctree(NormalOcTree octree, Color color, double octreeResolution)
   {
      normalOctreeGraphic.addMesh(octree, octreeResolution, color);
   }

   public void addOctree(List<Plane3D> octreePlanes, Color color, double octreeResolution)
   {
      normalOctreeGraphic.addMesh(octreePlanes, octreeResolution, color);
   }
   
   public void addOctree(List<IhmcSurfaceElement> surfaceElements, Color color)
   {
      normalOctreeGraphic.addMesh(surfaceElements, color);
   }

   public void addPlanarRegions(PlanarRegionsList planarRegions)
   {
      PlanarRegionsGraphic planarRegionsGraphic = new PlanarRegionsGraphic();
      planarRegionsGraphic.generateMeshes(planarRegions);
      planarRegionsGraphic.update();
      planarRegionsGraphics.add(planarRegionsGraphic);
   }

   public void start(String title)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            View3DFactory view3dFactory = new View3DFactory(1200, 800);
            view3dFactory.addCameraController(0.05, 2000.0, true);
            view3dFactory.addWorldCoordinateSystem(0.3);
            view3dFactory.addDefaultLighting();

            for (PlanarRegionsGraphic regionsGraphic : planarRegionsGraphics)
            {
               view3dFactory.addNodeToView(regionsGraphic);
            }

            stereoVisionPointCloudGraphic.generateMeshes();
            stereoVisionPointCloudGraphic.update();
            view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);

            normalOctreeGraphic.generateMeshes();
            normalOctreeGraphic.update();
            view3dFactory.addNodeToView(normalOctreeGraphic);

            Stage stage = new Stage();
            stage.setTitle(title);
            stage.setMaximized(false);
            stage.setScene(view3dFactory.getScene());

            stage.show();
         }
      });
   }
}
