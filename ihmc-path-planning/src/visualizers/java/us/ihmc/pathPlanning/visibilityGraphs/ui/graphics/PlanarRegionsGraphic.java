package us.ihmc.pathPlanning.visibilityGraphs.ui.graphics;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.JavaFXGraphicTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javafx.IdMappedColorFunction;
import us.ihmc.javaFXToolkit.graphics.JavaFXLabelGraphic;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class PlanarRegionsGraphic extends Group
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

   public PlanarRegionsGraphic()
   {
      this(true);
   }

   public PlanarRegionsGraphic(boolean initializeToFlatGround)
   {
      PlanarRegionsList planarRegionsList;
      if (initializeToFlatGround)
      {
         planarRegionsList = PlanarRegionsList.flatGround(20.0);
      }
      else
      {
         planarRegionsList = new PlanarRegionsList();
      }

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

      meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);

      double totalArea = 0.0;
      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon);

         totalArea += convexPolygon.getArea();
      }

      JavaFXLabelGraphic sizeLabel = null;
      if (drawAreaText)
      {
         sizeLabel = new JavaFXLabelGraphic(FormattingTools.getFormattedToSignificantFigures(totalArea, 3));
         sizeLabel.getPose().appendTransform(transformToWorld);
         sizeLabel.update();
      }

      if (drawBoundingBox)
      {
         JavaFXGraphicTools.drawBoxEdges(meshBuilder, PlanarRegionTools.getLocalBoundingBox3DInWorld(planarRegion, 0.1), 0.005);
      }

      if (drawNormal)
      {
         Point3DReadOnly centroid = PlanarRegionTools.getCentroid3DInWorld(planarRegion);

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

   public void setDrawAreaText(boolean drawAreaText)
   {
      this.drawAreaText = drawAreaText;
   }

   public void setDrawBoundingBox(boolean drawBoundingBox)
   {
      this.drawBoundingBox = drawBoundingBox;
   }

   public void setDrawNormal(boolean drawNormal)
   {
      this.drawNormal = drawNormal;
   }

   public void setColorFunction(Function<Integer, Color> colorFunction)
   {
      this.colorFunction = colorFunction;
   }
}
