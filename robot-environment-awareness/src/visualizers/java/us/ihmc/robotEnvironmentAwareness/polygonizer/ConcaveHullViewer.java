package us.ihmc.robotEnvironmentAwareness.polygonizer;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryGraphicsTools.*;
import static us.ihmc.robotEnvironmentAwareness.geometry.REAGraphics3DTools.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;

public class ConcaveHullViewer extends AnimationTimer
{
   private final BooleanProperty hideAll = new SimpleBooleanProperty(this, "hideAll", false);
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

   private final DoubleProperty borderVerticesSize = new SimpleDoubleProperty(this, "borderVerticesSize", 0.003);
   private final DoubleProperty borderEdgesSize = new SimpleDoubleProperty(this, "borderEdgesSize", 0.0015);
   private final ObjectProperty<Color> constraintEdgeColor = new SimpleObjectProperty<Color>(this, "constraintEdgeColor", Color.BLACK);
   private final DoubleProperty constraintEdgeSize = new SimpleDoubleProperty(this, "constraintEdgeSize", 0.002);

   private final AtomicReference<ConcaveHullViewerInput> latestInputReference = new AtomicReference<>(null);
   private final Random random = new Random(34534);

   public ConcaveHullViewer()
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

   public void submit(ConcaveHullViewerInput input)
   {
      latestInputReference.set(input);
      refreshMeshes();
   }

   public void submit(Color defaultColor, RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      latestInputReference.set(new ConcaveHullViewerInput(defaultColor, transformToWorld, concaveHullFactoryResult));
      refreshMeshes();
   }

   private void refreshMeshes()
   {
      ConcaveHullViewerInput latestInput = latestInputReference.get();
      if (latestInput == null)
         return;

      Color defaultColor = latestInput.defaultColor;
      RigidBodyTransform transformToWorld = latestInput.transformToWorld;
      ConcaveHullFactoryResult concaveHullFactoryResult = latestInput.concaveHullFactoryResult;

      graphicDelaunayTriangulation.submitForRendering(delaunayTrianglesToRainbowTriangles(transformToWorld, concaveHullFactoryResult, random));
      graphicBorderVertices.submitForRendering(borderVerticesToMultiSpheres(transformToWorld, concaveHullFactoryResult, defaultColor,
                                                                            borderVerticesSize.get()));
      graphicBorderEdges.submitForRendering(borderEdgesToMultiLine(transformToWorld, concaveHullFactoryResult, defaultColor, borderEdgesSize.get()));
      graphicBorderTriangles.submitForRendering(borderTrianglesToRainbowMultiTriangles(transformToWorld, concaveHullFactoryResult, random));
      graphicConstraintEdges.submitForRendering(constraintEdgesToMultiLine(transformToWorld, concaveHullFactoryResult, constraintEdgeColor.get(),
                                                                           constraintEdgeSize.get()));
      graphicOrderedBorderEdges.submitForRendering(orderedBorderEdgesToRainbowMultiLine(transformToWorld, concaveHullFactoryResult));
      // TODO Implement priority queue viz
      graphicConcaveHull.submitForRendering(multiLine(transformToWorld, concaveHullFactoryResult.getConcaveHullCollection(), defaultColor,
                                                      borderEdgesSize.get()));
      // TODO Implement concave pockets viz
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public BooleanProperty hideAllProperty()
   {
      return hideAll;
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

   private class ViewerGraphicObject
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
         if (!viewGraphic.get() || hideAll.get())
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

   public static class ConcaveHullViewerInput
   {
      private Color defaultColor;
      private RigidBodyTransform transformToWorld;
      private ConcaveHullFactoryResult concaveHullFactoryResult;

      public ConcaveHullViewerInput(Color defaultColor, RigidBodyTransform transformToWorld, ConcaveHullFactoryResult input)
      {
         this.defaultColor = defaultColor;
         this.transformToWorld = transformToWorld;
         this.concaveHullFactoryResult = input;
      }
   }
}
