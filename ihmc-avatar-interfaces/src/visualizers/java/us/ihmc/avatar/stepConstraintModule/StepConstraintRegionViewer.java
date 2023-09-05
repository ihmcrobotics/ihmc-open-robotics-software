package us.ihmc.avatar.stepConstraintModule;

import com.google.common.util.concurrent.AtomicDouble;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javafx.IdMappedColorFunction;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class StepConstraintRegionViewer
{
   private static final boolean VERBOSE = false;
   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<List<MeshView>> graphicsToRender = new AtomicReference<>(null);
   private List<MeshView> graphicsRendered = null;

   private final AtomicDouble opacity = new AtomicDouble(1.0);

   private final AnimationTimer renderMeshAnimation;
   private final AtomicBoolean show = new AtomicBoolean(true);

   public StepConstraintRegionViewer(Messager messager, Topic<List<StepConstraintRegion>> stepConstraintRegionDataTopc, Topic<Boolean> showStepConstraintRegionsTopic)
   {
      this();
      messager.addTopicListener(stepConstraintRegionDataTopc, this::buildMeshAndMaterialOnThread);
      messager.addTopicListener(showStepConstraintRegionsTopic, this::setVisibile);
   }

   public StepConstraintRegionViewer()
   {
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
               if (VERBOSE)
                  LogTools.info("Rendering new planar regions.");
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

   public void buildMeshAndMaterialOnThread(List<StepConstraintRegion> stepConstraintRegions)
   {
      executorService.submit(() -> buildMeshAndMaterial(stepConstraintRegions));
   }

   public void setVisibile(boolean visible)
   {
      this.show.set(visible);
   }

   private void buildMeshAndMaterial(List<StepConstraintRegion> stepConstraintRegions)
   {
      if (VERBOSE)
         LogTools.info("Creating mesh and material for new planar regions.", this);

      List<MeshView> regionMeshViews = new ArrayList<>();

      for (int regionIndex = 0; regionIndex < stepConstraintRegions.size(); regionIndex++)
      {
         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
         StepConstraintRegion planarRegion = stepConstraintRegions.get(regionIndex);

         int regionId = planarRegion.getRegionId();
         RigidBodyTransform transformToWorld = new RigidBodyTransform(planarRegion.getTransformToWorld());

         meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull().getVertexBufferView(), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);
         meshBuilder.addPolygon(transformToWorld, planarRegion.getConvexHullInConstraintRegion());

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfHolesInRegion(); polygonIndex++)
         {
            ConcavePolygon2DReadOnly hole = planarRegion.getHoleInConstraintRegion(polygonIndex);
            meshBuilder.addMultiLine(transformToWorld, hole.getVertexBufferView(), VisualizationParameters.CONCAVEHULL_LINE_THICKNESS, true);


         }


         MeshView regionMeshView = new MeshView(meshBuilder.generateMesh());
         regionMeshView.setMaterial(new PhongMaterial(IdMappedColorFunction.INSTANCE.apply(regionId)));
         regionMeshViews.add(regionMeshView);
      }

      graphicsToRender.set(regionMeshViews);
   }

   public Node getRoot()
   {
      return root;
   }
}
