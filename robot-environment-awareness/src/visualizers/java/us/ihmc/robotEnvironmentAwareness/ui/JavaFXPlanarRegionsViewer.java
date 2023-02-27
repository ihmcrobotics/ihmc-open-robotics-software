package us.ihmc.robotEnvironmentAwareness.ui;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.atomic.AtomicReference;

public class JavaFXPlanarRegionsViewer
{
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private final MeshView planarRegionMeshView = new MeshView();

   private final AnimationTimer renderer;

   public JavaFXPlanarRegionsViewer()
   {
      TextureColorPalette2D colorPalette = new TextureColorPalette2D();
      colorPalette.setHueBrightnessBased(0.9);
      meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      renderer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            render();
         }
      };
   }

   public void start()
   {
      renderer.start();
   }

   public void stop()
   {
      renderer.stop();
   }

   public void clear()
   {
      meshAndMaterialToRender.set(new Pair<>(null, null));
   }

   public void submitPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      if (planarRegionsListMessage != null)
         meshAndMaterialToRender.set(generateMeshAndMaterial(planarRegionsListMessage));
   }

   private void render()
   {
      Pair<Mesh, Material> meshAndMaterial = meshAndMaterialToRender.getAndSet(null);

      if (meshAndMaterial != null)
      {
         planarRegionMeshView.setMesh(meshAndMaterial.getKey());
         planarRegionMeshView.setMaterial(meshAndMaterial.getValue());
      }
   }

   private Pair<Mesh, Material> generateMeshAndMaterial(PlanarRegionsListMessage newMessage)
   {
      meshBuilder.clear();

      double lineWidth = 0.01;
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(newMessage);

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         int regionId = planarRegion.getRegionId();
         Color regionColor = getRegionColor(regionId);
         planarRegion.getTransformToWorld(transformToWorld);

         meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), lineWidth, regionColor, true);

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
         {
            ConvexPolygon2D convexPolygon2d = planarRegion.getConvexPolygon(polygonIndex);
            regionColor = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) polygonIndex / (double) planarRegion.getNumberOfConvexPolygons()));
            meshBuilder.addPolygon(transformToWorld, convexPolygon2d, regionColor);
         }
      }

      Material material = meshBuilder.generateMaterial();
      Mesh mesh = meshBuilder.generateMesh();

      return new Pair<>(mesh, material);
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   public Node getRootNode()
   {
      return planarRegionMeshView;
   }
}
