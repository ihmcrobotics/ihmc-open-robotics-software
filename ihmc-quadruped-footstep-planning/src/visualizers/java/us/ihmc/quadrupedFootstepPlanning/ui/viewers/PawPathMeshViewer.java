package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlan;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;


public class PawPathMeshViewer extends AnimationTimer
{
   private static final double DEFAULT_RADIUS = 0.05;
   private static final double zOffset = 0.01;
   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicBoolean showingSolution = new AtomicBoolean(false);
   private final AtomicBoolean showSolution = new AtomicBoolean(true);
   private final AtomicBoolean clearSolution = new AtomicBoolean(false);

   private final AtomicBoolean showingPreviewSteps = new AtomicBoolean(false);
   private final AtomicBoolean showPreviewSteps = new AtomicBoolean(false);
   private final AtomicBoolean clearPreviewSteps = new AtomicBoolean(false);

   private final AtomicBoolean solutionWasReceived = new AtomicBoolean(false);
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final QuadrantDependentList<Sphere> footstepPreviewSpheres = new QuadrantDependentList<>();
   private final QuadrantDependentList<Point3D> previewFootstepPositions = new QuadrantDependentList<>();

   private final MeshView footstepPathMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   public static final QuadrantDependentList<Color> defaultSolutionFootstepColors = new QuadrantDependentList<>(Color.GREEN, Color.RED, Color.DARKGREEN, Color.DARKRED);
   private static final QuadrantDependentList<Color> intermediateFootstepColors = new QuadrantDependentList<>(Color.rgb(160, 160, 160), Color.rgb(160, 160, 160), Color.rgb(160, 160, 160), Color.rgb(160, 160, 160));

   private final Messager messager;
   private final Topic<Boolean> showFootstepPlanTopic;
   private final Topic<Boolean> showFootstepPreviewTopic;

   private QuadrantDependentList<Color> solutionFootstepColors = defaultSolutionFootstepColors;
   private double footstepRadius = DEFAULT_RADIUS;

   public PawPathMeshViewer(Messager messager, Topic<PawPlan> footstepPlanTopic, Topic<Boolean> computePathTopic, Topic<Boolean> showFootstepPlanTopic,
                            Topic<Boolean> showFootstepPreviewTopic)
   {
      this.messager = messager;
      this.showFootstepPlanTopic = showFootstepPlanTopic;
      this.showFootstepPreviewTopic = showFootstepPreviewTopic;

      messager.registerTopicListener(footstepPlanTopic, footstepPlan -> executorService.submit(() -> {
         solutionWasReceived.set(true);
         processFootstepPath(footstepPlan);
      }));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Sphere previewSphere = new Sphere(footstepRadius);
         previewSphere.setMouseTransparent(true);

         Point3D position = new Point3D();
         position.setToNaN();

         previewFootstepPositions.put(robotQuadrant, position);
         footstepPreviewSpheres.put(robotQuadrant, previewSphere);
         root.getChildren().add(previewSphere);
      }

      messager.registerTopicListener(computePathTopic, data -> reset.set(true));
      messager.registerTopicListener(showFootstepPlanTopic, this::handleShowSolution);
      messager.registerTopicListener(showFootstepPreviewTopic, this::handleShowPreview);
   }

   public void setFootstepColors(QuadrantDependentList<Color> colors)
   {
      solutionFootstepColors = colors;
   }

   public void setFootstepRadius(double radius)
   {
      this.footstepRadius = radius;
   }

   public QuadrantDependentList<Color> getSolutionFootstepColors()
   {
      return solutionFootstepColors;
   }

   public QuadrantDependentList<Point3D> getPreviewFootstepPositions()
   {
      return previewFootstepPositions;
   }

   private void handleShowSolution(boolean show)
   {
      showSolution.set(show);
      if (show)
         messager.submitMessage(showFootstepPreviewTopic, false);
      else
         clearSolution.set(true);
   }

   private void handleShowPreview(boolean show)
   {
      showPreviewSteps.set(show);
      if (show)
         messager.submitMessage(showFootstepPlanTopic, false);
   }


   private synchronized void processFootstepPath(PawPlan plan)
   {
      meshBuilder.clear();
      QuadrantDependentList<Color> colors = solutionWasReceived.get() ? solutionFootstepColors : intermediateFootstepColors;

      if (plan == null)
      {
         reset.set(true);
         return;
      }
      FramePoint3D footPosition = new FramePoint3D();

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStep footstep = plan.getPawStep(i);
         Color regionColor = colors.get(footstep.getRobotQuadrant());

         footPosition.set(footstep.getGoalPosition());
         footPosition.addZ(zOffset);

         meshBuilder.addSphere(footstepRadius, footPosition, regionColor);
      }

      meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   @Override
   public void handle(long now)
   {
      boolean addSolution = showSolution.get() && solutionWasReceived.get() && !showingSolution.get();
      if (addSolution)
      {
         showingSolution.set(true);
         root.getChildren().add(footstepPathMeshView);
      }

      boolean addPreviewSteps = showPreviewSteps.get() && !showingPreviewSteps.get();
      if (addPreviewSteps)
      {
         showingPreviewSteps.set(true);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            root.getChildren().add(footstepPreviewSpheres.get(robotQuadrant));
      }

      boolean removeSolution = !showSolution.get() && solutionWasReceived.get() && clearSolution.get() && showingSolution.get();
      boolean removePreview = !showPreviewSteps.get() && clearPreviewSteps.get() && showingPreviewSteps.get();
      if (removeSolution || removePreview)
      {
         showingSolution.set(false);
         showingPreviewSteps.set(false);
         clearSolution.set(false);
         clearPreviewSteps.set(false);
         root.getChildren().clear();
      }

      if (reset.getAndSet(false))
      {
         footstepPathMeshView.setMesh(null);
         footstepPathMeshView.setMaterial(null);
         meshReference.set(null);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            previewFootstepPositions.get(robotQuadrant).setToNaN();
         return;
      }

      Pair<Mesh, Material> newMeshAndMaterial = meshReference.getAndSet(null);
      if (newMeshAndMaterial != null)
      {
         footstepPathMeshView.setMesh(newMeshAndMaterial.getKey());
         footstepPathMeshView.setMaterial(newMeshAndMaterial.getValue());
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Sphere previewSphere = footstepPreviewSpheres.get(robotQuadrant);
         Point3DReadOnly position = previewFootstepPositions.get(robotQuadrant);
         previewSphere.setTranslateX(position.getX());
         previewSphere.setTranslateY(position.getY());
         previewSphere.setTranslateZ(position.getZ() + 2 * zOffset);
      }
   }

   @Override
   public void stop()
   {
      super.stop();
      executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
