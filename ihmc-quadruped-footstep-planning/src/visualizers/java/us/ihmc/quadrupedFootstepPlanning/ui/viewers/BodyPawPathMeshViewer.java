package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PathTools;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters.BODYPATH_LINE_THICKNESS;

public class BodyPawPathMeshViewer extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private final double startColorHue = Color.GREEN.getHue();
   private final double goalColorHue = Color.RED.getHue();

   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();
   private final MeshView bodyPathMeshView = new MeshView();

   private final AtomicReference<List<? extends Point3DReadOnly>> activeBodyPathReference = new AtomicReference<>(null);

   private final AtomicReference<Pair<Mesh, Material>> bodyPathMeshToRender = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicBoolean reset = new AtomicBoolean(false);


   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);

   public BodyPawPathMeshViewer(Messager messager, Topic<Boolean> showPathPathTopic, Topic<Boolean> computePathTopic, Topic<List<? extends Point3DReadOnly>> bodyPathDataTopic)
   {
      isExecutorServiceProvided = executorService == null;

      bodyPathMeshView.setMouseTransparent(true);
      bodyPathMeshView.setMaterial(new PhongMaterial(Color.YELLOW));

      Vector3D defaultSize = new Vector3D(1.0, 1.0, 1.0);
      defaultSize.scale(1.5 * BODYPATH_LINE_THICKNESS);

      show = messager.createInput(showPathPathTopic, true);
      messager.registerTopicListener(bodyPathDataTopic, this::processBodyPathOnThread);

      messager.registerTopicListener(computePathTopic, data -> reset.set(true));


      root.getChildren().addAll(bodyPathMeshView);
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && root.getChildren().isEmpty())
      {
         root.getChildren().addAll(bodyPathMeshView);
      }
      else if (!show.get() && !root.getChildren().isEmpty())
      {
         root.getChildren().clear();
      }

      if (reset.getAndSet(false))
      {
         bodyPathMeshView.setMesh(null);
         bodyPathMeshView.setMaterial(null);
         bodyPathMeshToRender.set(null);
         return;
      }

      Pair<Mesh, Material> newMesh = bodyPathMeshToRender.getAndSet(null);
      if (newMesh != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering body path line.");
         bodyPathMeshView.setMesh(newMesh.getKey());
         bodyPathMeshView.setMaterial(newMesh.getValue());
      }
   }

   private void processBodyPathOnThread(List<? extends Point3DReadOnly> bodyPath)
   {
      executorService.execute(() -> processBodyPath(bodyPath));
   }

   private void processBodyPath(List<? extends Point3DReadOnly> bodyPath)
   {
      if (bodyPath == null || bodyPath.isEmpty())
      {
         bodyPathMeshToRender.set(new Pair<>(null, null));
         activeBodyPathReference.set(null);
         if (VERBOSE)
            PrintTools.warn("Received body path that is null.");
         return;
      }

      // First let's make a deep copy for later usage.
      bodyPath = bodyPath.stream().map(Point3D::new).collect(Collectors.toList());

      if (VERBOSE)
         PrintTools.info(this, "Building mesh for body path.");

      double totalPathLength = PathTools.computePathLength(bodyPath);
      double currentLength = 0.0;

      palette.clearPalette();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

      for (int segmentIndex = 0; segmentIndex < bodyPath.size() - 1; segmentIndex++)
      {
         Point3DReadOnly lineStart = bodyPath.get(segmentIndex);
         Point3DReadOnly lineEnd = bodyPath.get(segmentIndex + 1);

         double lineStartHue = EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         currentLength += lineStart.distance(lineEnd);
         double lineEndHue = EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         meshBuilder.addLine(lineStart, lineEnd, BODYPATH_LINE_THICKNESS, Color.hsb(lineStartHue, 1.0, 0.5), Color.hsb(lineEndHue, 1.0, 1.0));
      }
      bodyPathMeshToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      activeBodyPathReference.set(bodyPath);
   }

   @Override
   public void stop()
   {
      super.stop();

      if (!isExecutorServiceProvided)
         executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
