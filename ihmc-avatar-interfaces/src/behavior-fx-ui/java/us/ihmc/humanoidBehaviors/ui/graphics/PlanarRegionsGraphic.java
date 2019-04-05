package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
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

   private volatile List<MeshView> regionMeshViews;

   public PlanarRegionsGraphic()
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();  // start with a flat ground region
      convexPolygon.addVertex(10.0, 10.0);
      convexPolygon.addVertex(-10.0, 10.0);
      convexPolygon.addVertex(-10.0, -10.0);
      convexPolygon.addVertex(10.0, -10.0);
      convexPolygon.update();
      PlanarRegion groundPlane = new PlanarRegion(new RigidBodyTransform(), convexPolygon);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(groundPlane);

      generateMeshes(planarRegionsList);
   }

   public void generateMeshes(PlanarRegionsList planarRegionsList)
   {
      List<MeshView> updateRegionMeshViews = new ArrayList<>();

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         int regionId = planarRegion.getRegionId();
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transformToWorld);

         meshBuilder.addMultiLine(transformToWorld, Arrays.asList(planarRegion.getConcaveHull()), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
         {
            ConvexPolygon2D convexPolygon2d = planarRegion.getConvexPolygon(polygonIndex);
            meshBuilder.addPolygon(transformToWorld, convexPolygon2d);
         }

         MeshView regionMeshView = new MeshView(meshBuilder.generateMesh());
         regionMeshView.setMaterial(new PhongMaterial(getRegionColor(regionId)));
         updateRegionMeshViews.add(regionMeshView);
      }

      regionMeshViews = updateRegionMeshViews; // volatile set
   }

   public void update()
   {
      List<MeshView> meshViews = regionMeshViews;  // volatile get
      getChildren().clear();
      getChildren().addAll(meshViews);
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
