package us.ihmc.humanoidBehaviors.ui.graphics;

import com.google.common.util.concurrent.AtomicDouble;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarRegionsGraphic
{
   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<List<MeshView>> graphicsToRender = new AtomicReference<>(null);
   private List<MeshView> graphicsRendered = null;

   private final AtomicDouble opacity = new AtomicDouble(1.0);

   private final AnimationTimer renderMeshAnimation;
   private final AtomicReference<Boolean> show;

   private static final PlanarRegionColorPicker colorPicker = new PlanarRegionColorPicker();

   public PlanarRegionsGraphic(Messager messager, Topic<PlanarRegionsList> planarRegionDataTopic, Topic<Boolean> showPlanarRegionsTopic)
   {
      messager.registerTopicListener(planarRegionDataTopic, this::buildMeshAndMaterialOnThread);
      show = messager.createInput(showPlanarRegionsTopic, true);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (!show.get())
               root.getChildren().clear();               

            List<MeshView> localReference = graphicsToRender.getAndSet(null);

            if (localReference != null)
            {
               LogTools.debug("Rendering new planar regions.");
               graphicsRendered = localReference;
               root.getChildren().clear();
            }

            if (graphicsRendered != null && show.get() && root.getChildren().isEmpty())
               root.getChildren().addAll(graphicsRendered);
         }
      };
   }

   public void setOpacity(double newOpacity)
   {
      opacity.set(newOpacity);
   }

   public void start()
   {
      renderMeshAnimation.start();
   }

   public void stop()
   {
      renderMeshAnimation.stop();
      executorService.shutdownNow();
   }

   private void buildMeshAndMaterialOnThread(PlanarRegionsList planarRegionsList)
   {
      executorService.submit(() -> buildMeshAndMaterial(planarRegionsList));
   }

   private void buildMeshAndMaterial(PlanarRegionsList planarRegionsList)
   {
      LogTools.debug("Creating mesh and material for new planar regions.", this);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      List<MeshView> regionMeshViews = new ArrayList<>();

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         int regionId = planarRegion.getRegionId();
         planarRegion.getTransformToWorld(transformToWorld);

         meshBuilder.addMultiLine(transformToWorld, Arrays.asList(planarRegion.getConcaveHull()), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
         {
            ConvexPolygon2D convexPolygon2d = planarRegion.getConvexPolygon(polygonIndex);
            meshBuilder.addPolygon(transformToWorld, convexPolygon2d);
         }

         MeshView regionMeshView = new MeshView(meshBuilder.generateMesh());
         regionMeshView.setMaterial(new PhongMaterial(getRegionColor(regionId)));
         regionMeshViews.add(regionMeshView);
      }

      graphicsToRender.set(regionMeshViews);
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

   public Node getRoot()
   {
      return root;
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
