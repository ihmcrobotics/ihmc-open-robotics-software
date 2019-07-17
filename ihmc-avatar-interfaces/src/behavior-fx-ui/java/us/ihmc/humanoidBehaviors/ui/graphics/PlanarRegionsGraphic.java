package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PlanarRegionsGraphic extends Group
{
   private static final PlanarRegionColorPicker colorPicker = new PlanarRegionColorPicker();
   private static final boolean SHOW_AREA = false;

   private volatile List<Node> regionNodes;
   private List<Node> lastNodes = null; // optimization

   private Object regionMeshAddSync = new Object(); // for parallel mesh builder
   private volatile List<Node> updateRegionMeshViews; // for parallel mesh builder

   public PlanarRegionsGraphic()
   {
      this(true);
   }

   public PlanarRegionsGraphic(boolean initializeToFlatGround)
   {
      PlanarRegionsList planarRegionsList;
      if (initializeToFlatGround)
      {
         ConvexPolygon2D convexPolygon = new ConvexPolygon2D();  // start with a flat ground region
         convexPolygon.addVertex(10.0, 10.0);
         convexPolygon.addVertex(-10.0, 10.0);
         convexPolygon.addVertex(-10.0, -10.0);
         convexPolygon.addVertex(10.0, -10.0);
         convexPolygon.update();
         PlanarRegion groundPlane = new PlanarRegion(new RigidBodyTransform(), convexPolygon);
         planarRegionsList = new PlanarRegionsList(groundPlane);
      }
      else
      {
         planarRegionsList = new PlanarRegionsList();
      }

      generateMeshes(planarRegionsList);
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

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      meshBuilder.addMultiLine(transformToWorld, Arrays.asList(planarRegion.getConcaveHull()), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);

      double totalArea = 0.0;
      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon);

         totalArea += convexPolygon.getArea();
      }

      LabelGraphic sizeLabel;
      if (SHOW_AREA)
      {
         sizeLabel = new LabelGraphic(FormattingTools.getFormattedToSignificantFigures(totalArea, 3));
         sizeLabel.getPose().appendTransform(transformToWorld);
         sizeLabel.update();
      }

      MeshView regionMeshView = new MeshView(meshBuilder.generateMesh());
      regionMeshView.setMaterial(new PhongMaterial(getRegionColor(planarRegion.getRegionId())));

      synchronized (regionMeshAddSync)
      {
         if (SHOW_AREA) updateRegionMeshViews.add(sizeLabel.getNode());
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

   public static Color getRegionColor(int regionId)
   {
      return getRegionColor(regionId, 1.0);
   }

   public static Color getRegionColor(int regionId, double opacity)
   {
      java.awt.Color awtColor = colorPicker.getColor(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue(), opacity);
   }

   /**
    * Keeps a list N of good colors to render planar regions. Region i is given color i mod N
    */
   private static class PlanarRegionColorPicker
   {
      private final ArrayList<java.awt.Color> colors = new ArrayList<>();

      PlanarRegionColorPicker()
      {
         colors.add(new java.awt.Color(104, 130, 219));
         colors.add(new java.awt.Color(113, 168, 133));
         colors.add(new java.awt.Color(196, 182, 90));
         colors.add(new java.awt.Color(190, 89, 110));
         colors.add(new java.awt.Color(155, 80, 190));
      }

      java.awt.Color getColor(int regionId)
      {
         return colors.get(Math.abs(regionId % colors.size()));
      }
   }
}
