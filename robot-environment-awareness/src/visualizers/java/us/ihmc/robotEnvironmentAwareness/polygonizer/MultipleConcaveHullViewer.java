package us.ihmc.robotEnvironmentAwareness.polygonizer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.polygonizer.ConcaveHullViewer.ConcaveHullViewerInput;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;

public class MultipleConcaveHullViewer extends AnimationTimer
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme(MultipleConcaveHullViewer.class.getSimpleName()));

   private static final CategoryTheme ConcaveHullFactory = apiFactory.createCategoryTheme("ConcaveHullFactory");
   private static final CategoryTheme DelaunayTriangulation = apiFactory.createCategoryTheme("DelaunayTriangulation");
   private static final CategoryTheme Border = apiFactory.createCategoryTheme("Border");
   private static final CategoryTheme Edge = apiFactory.createCategoryTheme("Edge");
   private static final CategoryTheme Triangle = apiFactory.createCategoryTheme("Triangle");
   private static final CategoryTheme Vertex = apiFactory.createCategoryTheme("Vertex");
   private static final CategoryTheme Constraint = apiFactory.createCategoryTheme("Constraint");
   private static final CategoryTheme Ordered = apiFactory.createCategoryTheme("Ordered");
   private static final CategoryTheme PriorityQueue = apiFactory.createCategoryTheme("PriorityQueue");
   private static final CategoryTheme Concave = apiFactory.createCategoryTheme("Concave");
   private static final CategoryTheme Hull = apiFactory.createCategoryTheme("Hull");
   private static final CategoryTheme Pocket = apiFactory.createCategoryTheme("Pocket");
   private static final CategoryTheme Raw = apiFactory.createCategoryTheme("Raw");
   private static final CategoryTheme Processed = apiFactory.createCategoryTheme("Processed");

   private static final TypedTopicTheme<Boolean> View = apiFactory.createTypedTopicTheme("view");

   public static final Topic<Boolean> ViewDelaunayTriangulation = Root.child(ConcaveHullFactory).child(DelaunayTriangulation).topic(View);
   public static final Topic<Boolean> ViewBorderVertices = Root.child(ConcaveHullFactory).child(Border).child(Vertex).topic(View);
   public static final Topic<Boolean> ViewBorderEdges = Root.child(ConcaveHullFactory).child(Border).child(Edge).topic(View);
   public static final Topic<Boolean> ViewBorderTriangles = Root.child(ConcaveHullFactory).child(Border).child(Triangle).topic(View);
   public static final Topic<Boolean> ViewConstraintEdges = Root.child(ConcaveHullFactory).child(Constraint).child(Edge).topic(View);
   public static final Topic<Boolean> ViewOrderedBorderEdges = Root.child(ConcaveHullFactory).child(Ordered).child(Border).child(Edge).topic(View);
   public static final Topic<Boolean> ViewPriorityQueue = Root.child(ConcaveHullFactory).child(PriorityQueue).topic(View);
   public static final Topic<Boolean> ViewRawConcaveHull = Root.child(ConcaveHullFactory).child(Concave).child(Hull).child(Raw).topic(View);
   public static final Topic<Boolean> ViewProcessedConcaveHull = Root.child(ConcaveHullFactory).child(Concave).child(Hull).child(Processed).topic(View);
   public static final Topic<Boolean> ViewConcavePockets = Root.child(ConcaveHullFactory).child(Concave).child(Pocket).topic(View);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private final Group rootNode = new Group();
   private final List<ConcaveHullViewer> concaveHullViewerRendered = new ArrayList<>();
   private final AtomicReference<List<ConcaveHullViewer>> concaveHullViewerToRender = new AtomicReference<>(null);

   private final BooleanProperty showDelaunayTriangulation = new SimpleBooleanProperty(this, "viewDelaunayTriangulation", true);
   private final BooleanProperty showBorderVertices = new SimpleBooleanProperty(this, "viewBorderVertices", false);
   private final BooleanProperty showBorderEdges = new SimpleBooleanProperty(this, "viewBorderEdges", false);
   private final BooleanProperty showBorderTriangles = new SimpleBooleanProperty(this, "viewBorderTriangles", false);
   private final BooleanProperty showConstraintEdges = new SimpleBooleanProperty(this, "viewConstraintEdges", false);
   private final BooleanProperty showOrderedBorderEdges = new SimpleBooleanProperty(this, "viewOrderedBorderEdges", true);
   private final BooleanProperty showPriorityQueue = new SimpleBooleanProperty(this, "viewPriorityQueue", false);
   private final BooleanProperty showRawConcaveHull = new SimpleBooleanProperty(this, "viewConcaveHull", false);
   private final BooleanProperty showConcavePockets = new SimpleBooleanProperty(this, "viewConcavePockets", false);
   private final BooleanProperty showProcessedConcaveHull = new SimpleBooleanProperty(this, "viewConcaveHull", false);

   public MultipleConcaveHullViewer(JavaFXMessager messager)
   {
      messager.bindBidirectional(ViewDelaunayTriangulation, showDelaunayTriangulation, true);
      messager.bindBidirectional(ViewBorderVertices, showBorderVertices, true);
      messager.bindBidirectional(ViewBorderEdges, showBorderEdges, true);
      messager.bindBidirectional(ViewBorderTriangles, showBorderTriangles, true);
      messager.bindBidirectional(ViewConstraintEdges, showConstraintEdges, true);
      messager.bindBidirectional(ViewOrderedBorderEdges, showOrderedBorderEdges, true);
      messager.bindBidirectional(ViewPriorityQueue, showPriorityQueue, true);
      messager.bindBidirectional(ViewRawConcaveHull, showRawConcaveHull, true);
      messager.bindBidirectional(ViewConcavePockets, showConcavePockets, true);
      messager.bindBidirectional(ViewProcessedConcaveHull, showProcessedConcaveHull, true);
   }

   @Override
   public void handle(long now)
   {
      List<ConcaveHullViewer> newViewers = concaveHullViewerToRender.getAndSet(null);

      if (newViewers != null)
      {
         rootNode.getChildren().clear();
         concaveHullViewerRendered.clear();
         concaveHullViewerRendered.addAll(newViewers);
         for (ConcaveHullViewer newViewer : newViewers)
         {
            rootNode.getChildren().add(newViewer.getRootNode());
            newViewer.viewDelaunayTriangulationProperty().bindBidirectional(showDelaunayTriangulation);
            newViewer.viewBorderVerticesProperty().bindBidirectional(showBorderVertices);
            newViewer.viewBorderEdgesProperty().bindBidirectional(showBorderEdges);
            newViewer.viewBorderTrianglesProperty().bindBidirectional(showBorderTriangles);
            newViewer.viewConstraintEdgesProperty().bindBidirectional(showConstraintEdges);
            newViewer.viewOrderedBorderEdgesProperty().bindBidirectional(showOrderedBorderEdges);
            newViewer.viewPriorityQueueProperty().bindBidirectional(showPriorityQueue);
            newViewer.viewRawConcaveHullProperty().bindBidirectional(showRawConcaveHull);
            newViewer.viewConcavePocketsProperty().bindBidirectional(showConcavePockets);
            newViewer.viewProcessedConcaveHullProperty().bindBidirectional(showProcessedConcaveHull);
         }
      }

      concaveHullViewerRendered.forEach(viewer -> viewer.handle(now));
   }

   public void submit(Collection<ConcaveHullViewerInput> inputs)
   {
      List<ConcaveHullViewer> newViewers = new ArrayList<>();

      for (ConcaveHullViewerInput input : inputs)
      {
         ConcaveHullViewer concaveHullViewer = new ConcaveHullViewer();
         concaveHullViewer.submit(input);
         newViewers.add(concaveHullViewer);
      }

      concaveHullViewerToRender.set(newViewers);
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public static List<ConcaveHullViewerInput> toConcaveHullViewerInputList(Collection<Polygonizer.Output> polygonizerOutputs)
   {
      return polygonizerOutputs.stream().map(MultipleConcaveHullViewer::toConcaveHullViewerInput).collect(Collectors.toList());
   }

   public static ConcaveHullViewerInput toConcaveHullViewerInput(Polygonizer.Output polygonizerOutput)
   {
      Color color = OcTreeMeshBuilder.getRegionColor(polygonizerOutput.getInput().getId());
      RigidBodyTransform transformToWorld = polygonizerOutput.getInput().getTransformToWorld();
      ConcaveHullFactoryResult concaveHullFactoryResult = polygonizerOutput.getConcaveHullFactoryResult();
      ConcaveHullCollection processedConcaveHullCollection = polygonizerOutput.getProcessedConcaveHullCollection();
      return new ConcaveHullViewerInput(color, transformToWorld, concaveHullFactoryResult, processedConcaveHullCollection);
   }
}
