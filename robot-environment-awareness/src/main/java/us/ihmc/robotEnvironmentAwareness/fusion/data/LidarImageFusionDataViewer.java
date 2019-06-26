package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class LidarImageFusionDataViewer
{
   private final Messager messager;

   private final AtomicReference<LidarImageFusionData> lidarImageFusionDataToRender;

   protected final JavaFXMultiColorMeshBuilder meshBuilder;

   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>(null);
   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();
   private final AtomicReference<Boolean> clear = new AtomicReference<>(false);

   // TODO: just for debugging.
   private final AtomicReference<PlanarRegionPropagationParameters> planarRegionPropagationParameters;

   public LidarImageFusionDataViewer(SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      lidarImageFusionDataToRender = messager.createInput(LidarImageFusionAPI.FusionDataState, null);

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      messager.registerTopicListener(LidarImageFusionAPI.ShowFusionData, (content) -> unpackFusionData());
      planarRegionPropagationParameters = messager.createInput(LidarImageFusionAPI.PlanarRegionPropagationParameters, new PlanarRegionPropagationParameters());
   }

   private void unpackFusionData()
   {
      clear();
      double lineWidth = 0.01;
      meshBuilder.clear();
      LidarImageFusionData lidarImageFusionData = lidarImageFusionDataToRender.get();

      if (lidarImageFusionData == null)
         return;

      int numberOfSegment = lidarImageFusionData.getNumberOfImageSegments();
      System.out.println("LidarImageFusionDataViewer numberOfSegment " + numberOfSegment);

      List<PlanarRegionSegmentationRawData> planarRegionSegmentationRawDataList = new ArrayList<>();

      for (int i = 0; i < numberOfSegment; i++)
      {
         SegmentationRawData data = lidarImageFusionData.getFusionDataSegment(i);
         SegmentationNodeData segmentationNodeData = new SegmentationNodeData(data);

         PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(i, segmentationNodeData.getNormal(),
                                                                                                               segmentationNodeData.getCenter(),
                                                                                                               segmentationNodeData.getPointsInSegment());

         planarRegionSegmentationRawDataList.add(planarRegionSegmentationRawData);
      }

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

      PlanarRegionsList planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(planarRegionSegmentationRawDataList, concaveHullFactoryParameters,
                                                                                            polygonizerParameters);

      System.out.println("LidarImageFusionDataViewer planarRegionsList " + planarRegionsList.getNumberOfPlanarRegions());

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         int randomID = new Random().nextInt();
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transformToWorld);

         Color regionColor = getRegionColor(randomID);
         
         //         meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), lineWidth, regionColor, true);
         //         meshBuilder.addTetrahedron(0.02, transformToWorld.getTranslationVector(), regionColor);
      }

      for (int i = 0; i < numberOfSegment; i++)
      {
         int randomID = new Random().nextInt();
         Color regionColor = getRegionColor(randomID);
         SegmentationRawData data = lidarImageFusionData.getFusionDataSegment(i);
         Point3D center = data.getCenter();
         Vector3D normal = data.getNormal();
         Point3D centerEnd = new Point3D(normal);
         centerEnd.scaleAdd(0.1, center);
         if (data.isSparse())
         {
            regionColor = Color.rgb(0, 0, 0);
         }
         
         meshBuilder.addLine(center, centerEnd, lineWidth, regionColor);
         for (Point3D point : data.getPoints())
            meshBuilder.addTetrahedron(0.02, point, regionColor);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      meshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   public void render()
   {
      MeshView newScanMeshView = meshToRender.getAndSet(null);

      if (clear.getAndSet(false))
         children.clear();

      if (newScanMeshView != null)
      {
         children.add(newScanMeshView);
      }
   }

   public void clear()
   {
      clear.set(true);
   }

   public Node getRoot()
   {
      return root;
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }
}
