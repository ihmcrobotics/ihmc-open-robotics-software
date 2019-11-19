package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

//TODO: replace each graphic node to stereo vision one.
public class StereoVisionPointCloudGraphic extends Group
{
   private volatile List<Node> regionNodes;
   private List<Node> lastNodes = null; // optimization

   private Object regionMeshAddSync = new Object(); // for parallel mesh builder
   private volatile List<Node> updateRegionMeshViews; // for parallel mesh builder

   // visualization options
   private boolean drawAreaText = false;
   private boolean drawBoundingBox = false;
   private boolean drawNormal;

   private Function<Integer, Color> colorFunction = new IdMappedColorFunction();

   public StereoVisionPointCloudGraphic()
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();

      generateMeshes(planarRegionsList);
   }

   public void generateMeshesAsync(PlanarRegionsList planarRegionsList)
   {
      ThreadTools.startAThread(() -> generateMeshes(planarRegionsList), "MeshGeneration");
   }

   public synchronized void generateMeshes(PlanarRegionsList planarRegionsList)
   {
      updateRegionMeshViews = new ArrayList<>();

      planarRegionsList.getPlanarRegionsAsList().parallelStream().forEach(this::parallelMeshBuilder);

      regionNodes = updateRegionMeshViews; // volatile set
   }

   private void parallelMeshBuilder(PlanarRegion planarRegion)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      RigidBodyTransform transformToWorld = planarRegion.getTransformToWorldCopy();

      meshBuilder.addMultiLine(transformToWorld, Arrays.asList(planarRegion.getConcaveHull()), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);

      double totalArea = 0.0;
      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon);

         totalArea += convexPolygon.getArea();
      }

      LabelGraphic sizeLabel = null;
      if (drawAreaText)
      {
         sizeLabel = new LabelGraphic(FormattingTools.getFormattedToSignificantFigures(totalArea, 3));
         sizeLabel.getPose().appendTransform(transformToWorld);
         sizeLabel.update();
      }

      if (drawBoundingBox)
      {
         JavaFXGraphicTools.drawBoxEdges(meshBuilder, PlanarRegionTools.getLocalBoundingBox3DInWorld(planarRegion, 0.1), 0.005);
      }

      if (drawNormal)
      {
         Vector3D normal = planarRegion.getNormal();
         normal.normalize();

         Point3D centroid = PlanarRegionTools.getAverageCentroid3DInWorld(planarRegion);

         double length = 0.07;
         double radius = 0.004;
         double cylinderToConeLengthRatio = 0.8;
         double coneDiameterMultiplier = 1.8;
         JavaFXGraphicTools.drawArrow(meshBuilder,
                                      centroid,
                                      transformToWorld.getRotation(),
                                      length,
                                      radius,
                                      cylinderToConeLengthRatio,
                                      coneDiameterMultiplier);
      }

      MeshView regionMeshView = new MeshView(meshBuilder.generateMesh());
      regionMeshView.setMaterial(new PhongMaterial(colorFunction.apply(planarRegion.getRegionId())));

      synchronized (regionMeshAddSync)
      {
         if (drawAreaText) updateRegionMeshViews.add(sizeLabel.getNode());
         updateRegionMeshViews.add(regionMeshView);
      }
   }

   public void update()
   {
      List<Node> meshViews = regionNodes;  // volatile get
      if (lastNodes != meshViews) // optimization
      {
         getChildren().clear();
         getChildren().addAll(meshViews);
         lastNodes = meshViews;
      }
   }
}
