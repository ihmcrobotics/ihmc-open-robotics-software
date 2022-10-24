package us.ihmc.footstepPlanning.ui.viewers;

import com.google.common.util.concurrent.AtomicDouble;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javafx.MeshHolder;
import us.ihmc.messager.Messager;

import java.util.concurrent.atomic.AtomicReference;

public class BodyPathLogRenderer extends AnimationTimer
{
   private static final Color startNodeColor = Color.color(0.1, 0.1, 0.1, 0.9);
   private static final Color candidateNodeColor = Color.color(0.9, 0.9, 0.9, 0.8);

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());

   private final MeshHolder startNodeGraphic = new MeshHolder(root);
   private final MeshHolder candidateNodeGraphic = new MeshHolder(root);

   private final AtomicReference<Pair<BodyPathLatticePoint, Double>> startNodeToVisualize;
   private final AtomicReference<Pair<BodyPathLatticePoint, Double>> candidateNodeToVisualize;
   private AtomicDouble latestStanceHeight = new AtomicDouble();

   public BodyPathLogRenderer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.BodyPathStartNodeToVisualize, d -> latestStanceHeight.set(d.getRight()));
      startNodeToVisualize = messager.createInput(FootstepPlannerMessagerAPI.BodyPathStartNodeToVisualize);
      candidateNodeToVisualize = messager.createInput(FootstepPlannerMessagerAPI.BodyPathCandidateNodeToVisualize);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowBodyPathLogGraphics, root::setVisible);
      root.setVisible(false);
   }

   @Override
   public void handle(long now)
   {
      try
      {
         handleInternal();
      }
      catch (Exception e)
      {
         // don't hard crash if RuntimeException is thrown
         e.printStackTrace();
      }
   }

   private void handleInternal()
   {
      Pair<BodyPathLatticePoint, Double> startData = startNodeToVisualize.getAndSet(null);
      if (startData != null)
      {
         generateMesh(startData, startNodeGraphic, startNodeColor);
      }

      Pair<BodyPathLatticePoint, Double> candidateData = candidateNodeToVisualize.getAndSet(null);
      if (candidateData != null)
      {
         generateMesh(candidateData, candidateNodeGraphic, candidateNodeColor);
      }
   }

   public Group getRoot()
   {
      return root;
   }

   private void generateMesh(Pair<BodyPathLatticePoint, Double> data, MeshHolder meshHolder, Color color)
   {
      meshBuilder.clear();
      BodyPathLatticePoint node = data.getLeft();
      double height = data.getRight();

      if (Double.isNaN(height))
      {
         height = latestStanceHeight.get() + 0.1;
      }

      double bodyWidth = 0.05;
      meshBuilder.addBox(bodyWidth, bodyWidth, bodyWidth, new Point3D(node.getX(), node.getY(), height), color);

      meshHolder.setMeshReference(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      meshHolder.update();
   }

}
