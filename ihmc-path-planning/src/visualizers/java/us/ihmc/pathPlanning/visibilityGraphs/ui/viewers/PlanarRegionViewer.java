package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionViewer
{
   private static final boolean VERBOSE = true;
   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final MeshView planarRegionMeshView = new MeshView();

   private final AtomicReference<Pair<Mesh, Material>> graphicsToRender = new AtomicReference<>(null);
   private Pair<Mesh, Material> graphicsRendered = null;
   private final TextureColorPalette2D colorPalette = new TextureColorPalette2D();

   private final AnimationTimer renderMeshAnimation;

   public PlanarRegionViewer(REAMessager messager, Topic<PlanarRegionsList> planarRegionDataTopic, Topic<Boolean> showPlanarRegionsTopic)
   {
      colorPalette.setHueBrightnessBased(0.9);
      messager.registerTopicListener(planarRegionDataTopic, this::buildMeshAndMaterialOnThread);
      messager.registerTopicListener(showPlanarRegionsTopic, this::handleShowThreadSafe);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            Pair<Mesh, Material> localReference = graphicsToRender.getAndSet(null);

            if (localReference != null)
            {
               if (VERBOSE)
                  PrintTools.info(this, "Rendering new planar regions.");
               graphicsRendered = localReference;
               planarRegionMeshView.setMesh(localReference.getKey());
               planarRegionMeshView.setMaterial(localReference.getValue());
            }
         }
      };
   }

   private void handleShowThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShow(show);
      else
         Platform.runLater(() -> handleShow(show));
   }

   private void handleShow(boolean show)
   {
      if (!show)
      {
         planarRegionMeshView.setMesh(null);
      }
      else if (graphicsRendered != null)
      {
         planarRegionMeshView.setMesh(graphicsRendered.getKey());
         planarRegionMeshView.setMaterial(graphicsRendered.getValue());
      }
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
      if (VERBOSE)
         PrintTools.info(this, "Creating mesh and material for new planar regions.");
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();


      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         int regionId = planarRegion.getRegionId();
         Color regionColor = getRegionColor(regionId);
         planarRegion.getTransformToWorld(transformToWorld);

         meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, regionColor, true);

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
         {
            ConvexPolygon2D convexPolygon2d = planarRegion.getConvexPolygon(polygonIndex);
            regionColor = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) polygonIndex / (double) planarRegion.getNumberOfConvexPolygons()));
            meshBuilder.addPolygon(transformToWorld, convexPolygon2d, regionColor);
         }
      }

      Material material = meshBuilder.generateMaterial();
      Mesh mesh = meshBuilder.generateMesh();
      graphicsToRender.set(new Pair<>(mesh, material));
   }


   public static Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   public Node getRoot()
   {
      return planarRegionMeshView;
   }
}
