package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class BodyPathMeshViewer extends AnimationTimer
{
   private static final boolean VERBOSE = true;

   private final MeshView bodyPathMeshView = new MeshView();

   private final AtomicReference<Mesh> bodyPathMeshToRender = new AtomicReference<>(null);
   private Mesh bodyPathMeshRendered = null;
   private final AtomicReference<Boolean> resetRequested;

   public BodyPathMeshViewer(REAMessager messager)
   {
      bodyPathMeshView.setMouseTransparent(true);
      bodyPathMeshView.setMaterial(new PhongMaterial(Color.YELLOW));

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowBodyPath, this::handleShowThreadSafe);
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
         bodyPathMeshView.setMesh(null);
      else
         bodyPathMeshView.setMesh(bodyPathMeshRendered);
   }

   public void processBodyPath(List<Point3DReadOnly> bodyPath)
   {
      if (VERBOSE)
         PrintTools.info(this, "Building mesh for body path.");
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      meshBuilder.addMultiLine(bodyPath, VisualizationParameters.BODYPATH_LINE_THICKNESS, false);
      bodyPathMeshToRender.set(meshBuilder.generateMesh());
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         bodyPathMeshView.setMesh(null);
         bodyPathMeshToRender.getAndSet(null);
         return;
      }

      Mesh newMesh = bodyPathMeshToRender.getAndSet(null);

      if (newMesh != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering body path line.");
         bodyPathMeshRendered = newMesh;
         bodyPathMeshView.setMesh(newMesh);
      }
   }

   public Node getRoot()
   {
      return bodyPathMeshView;
   }
}
