package us.ihmc.robotEnvironmentAwareness;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryGraphicsTools.*;
import static us.ihmc.robotEnvironmentAwareness.geometry.REAGraphics3DTools.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;

public class ConcaveHullFactoryResultViewer extends AnimationTimer
{
   private final ViewerGraphicObject graphicDelaunayTriangulation = new ViewerGraphicObject("viewDelaunayTriangulation", true);
   private final ViewerGraphicObject graphicBorderVertices = new ViewerGraphicObject("viewBorderVertices", false);
   private final ViewerGraphicObject graphicBorderEdges = new ViewerGraphicObject("viewBorderEdges", false);
   private final ViewerGraphicObject graphicBorderTriangles = new ViewerGraphicObject("viewBorderTriangles", false);
   private final ViewerGraphicObject graphicConstraintEdges = new ViewerGraphicObject("viewConstraintEdges", false);
   private final ViewerGraphicObject graphicOrderedBorderEdges = new ViewerGraphicObject("viewOrderedBorderEdges", true);
   private final ViewerGraphicObject graphicPriorityQueue = new ViewerGraphicObject("viewPriorityQueue", false);
   private final ViewerGraphicObject graphicConcaveHull = new ViewerGraphicObject("viewConcaveHull", false);
   private final ViewerGraphicObject graphicConcavePockets = new ViewerGraphicObject("viewConcavePockets", false);
   private final List<ViewerGraphicObject> graphics = Arrays.asList(graphicDelaunayTriangulation, graphicBorderVertices, graphicBorderEdges,
                                                                    graphicBorderTriangles, graphicConstraintEdges, graphicOrderedBorderEdges,
                                                                    graphicPriorityQueue, graphicConcaveHull, graphicConcavePockets);

   private final Group rootNode = new Group();

   private double borderVerticesSize = 0.003;
   private double borderEdgesSize = 0.0015;
   private Color constraintEdgeColor = Color.BLACK;
   private double constraintEdgeSize = 0.002;

   private final AtomicReference<ViewerInput> latestInputReference = new AtomicReference<>(null);
   private final Random random = new Random(34534);

   public ConcaveHullFactoryResultViewer(JavaFXMessager messager)
   {
      graphics.forEach(graphic -> rootNode.getChildren().add(graphic.meshViewRendered));
      graphicBorderVertices.setMouseTransparent(true);
      graphicBorderEdges.setMouseTransparent(true);
      graphicOrderedBorderEdges.setMouseTransparent(true);
      graphicPriorityQueue.setMouseTransparent(true);
      graphicConcaveHull.setMouseTransparent(true);
   }

   @Override
   public void handle(long now)
   {
      graphics.forEach(graphic -> graphic.render());
   }

   public void submit(Color defaultColor, RigidBodyTransform transformToWorld, ConcaveHullFactoryResult input)
   {
      latestInputReference.set(new ViewerInput(defaultColor, transformToWorld, input));
      refreshMeshes();
   }

   private void refreshMeshes()
   {
      ViewerInput latestInput = latestInputReference.get();
      if (latestInput == null)
         return;

      Color defaultColor = latestInput.defaultColor;
      RigidBodyTransform transformToWorld = latestInput.transformToWorld;
      ConcaveHullFactoryResult concaveHullFactoryResult = latestInput.input;

      graphicDelaunayTriangulation.submitForRendering(delaunayTrianglesToRainbowTriangles(transformToWorld, concaveHullFactoryResult, random));
      graphicBorderVertices.submitForRendering(borderVerticesToMultiSpheres(transformToWorld, concaveHullFactoryResult, defaultColor, borderVerticesSize));
      graphicBorderEdges.submitForRendering(borderEdgesToMultiLine(transformToWorld, concaveHullFactoryResult, defaultColor, borderEdgesSize));
      graphicBorderTriangles.submitForRendering(borderTrianglesToRainbowMultiTriangles(transformToWorld, concaveHullFactoryResult, random));
      graphicConstraintEdges.submitForRendering(constraintEdgesToMultiLine(transformToWorld, concaveHullFactoryResult, constraintEdgeColor,
                                                                           constraintEdgeSize));
      graphicOrderedBorderEdges.submitForRendering(orderedBorderEdgesToRainbowMultiLine(transformToWorld, concaveHullFactoryResult));
      // TODO Implement priority queue viz
      graphicConcaveHull.submitForRendering(multiLine(transformToWorld, concaveHullFactoryResult.getConcaveHullCollection(), defaultColor, borderEdgesSize));
      // TODO Implement concave pockets viz
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public BooleanProperty viewDelaunayTriangulationProperty()
   {
      return graphicDelaunayTriangulation.viewGraphic;
   }

   public BooleanProperty viewBorderVerticesProperty()
   {
      return graphicBorderVertices.viewGraphic;
   }

   public BooleanProperty viewBorderEdgesProperty()
   {
      return graphicBorderEdges.viewGraphic;
   }

   public BooleanProperty viewBorderTrianglesProperty()
   {
      return graphicBorderTriangles.viewGraphic;
   }

   public BooleanProperty viewConstraintEdgesProperty()
   {
      return graphicConstraintEdges.viewGraphic;
   }

   public BooleanProperty viewOrderedBorderEdgesProperty()
   {
      return graphicOrderedBorderEdges.viewGraphic;
   }

   public BooleanProperty viewPriorityQueueProperty()
   {
      return graphicPriorityQueue.viewGraphic;
   }

   public BooleanProperty viewConcaveHullProperty()
   {
      return graphicConcaveHull.viewGraphic;
   }

   public BooleanProperty viewConcavePocketsProperty()
   {
      return graphicConcavePockets.viewGraphic;
   }

   private static class ViewerGraphicObject
   {
      private final BooleanProperty viewGraphic;
      private final MeshView meshViewRendered = new MeshView();
      private final AtomicReference<MeshView> meshViewToRenderRef = new AtomicReference<>(null);

      public ViewerGraphicObject(String name, boolean showInitialValue)
      {
         viewGraphic = new SimpleBooleanProperty(this, name, showInitialValue);
      }

      void render()
      {
         if (!viewGraphic.get())
         {
            meshViewRendered.setMesh(null);
            meshViewRendered.setMaterial(null);
            return;
         }

         MeshView meshViewToRender = meshViewToRenderRef.get();
         if (meshViewToRender != null)
         {
            meshViewRendered.setMesh(meshViewToRender.getMesh());
            meshViewRendered.setMaterial(meshViewToRender.getMaterial());
         }
      }

      void submitForRendering(MeshView newMeshViewToRender)
      {
         meshViewToRenderRef.set(newMeshViewToRender);
      }

      void setMouseTransparent(boolean value)
      {
         meshViewRendered.setMouseTransparent(value);
      }
   }

   private static class ViewerInput
   {
      private Color defaultColor;
      private RigidBodyTransform transformToWorld;
      private ConcaveHullFactoryResult input;

      public ViewerInput(Color defaultColor, RigidBodyTransform transformToWorld, ConcaveHullFactoryResult input)
      {
         this.defaultColor = defaultColor;
         this.transformToWorld = transformToWorld;
         this.input = input;
      }
   }
}
