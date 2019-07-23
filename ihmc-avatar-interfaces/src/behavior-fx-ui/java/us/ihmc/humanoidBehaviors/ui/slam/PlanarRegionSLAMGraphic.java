package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.scene.Group;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMParameters;
import us.ihmc.humanoidBehaviors.ui.graphics.MeshGraphic;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.lists.PairList;

import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import static us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMTools.*;
import static us.ihmc.humanoidBehaviors.ui.slam.PlanarRegionSLAMGraphic.SLAMVisualizationState.*;

public class PlanarRegionSLAMGraphic extends Group
{
   private final StateMachine<SLAMVisualizationState, State> stateMachine;
   private Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints;

   public enum SLAMVisualizationState
   {
      Hidden, BoundingBoxCollision
   }

   private PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::fxUpdate);
   private PlanarRegionsGraphic mapGraphic = new PlanarRegionsGraphic();
   private PlanarRegionsGraphic listToMergeGraphic = new PlanarRegionsGraphic();
   private MeshGraphic meshGraphic = new MeshGraphic(this::meshBuilder);

   private final PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
   private PlanarRegionsList map = new PlanarRegionsList();
   private PlanarRegionsList listToMerge = new PlanarRegionsList();

   private Notification nextStep = new Notification();

   private Consumer<SLAMVisualizationState> stateListener = state -> {};

   // intersection graphics here

   public PlanarRegionSLAMGraphic()
   {
      EnumBasedStateMachineFactory<SLAMVisualizationState> factory = new EnumBasedStateMachineFactory<>(SLAMVisualizationState.class);
      factory.addTransition(Hidden, BoundingBoxCollision, now -> nextStep.poll());
      factory.setOnEntry(BoundingBoxCollision, this::onBoundingBoxCollisionEntry);
      factory.addTransition(BoundingBoxCollision, Hidden, now -> nextStep.poll());
      factory.getFactory().addStateChangedListener((from, to) ->
      {
         LogTools.info("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
         stateListener.accept(to);
      });
      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(Hidden);
      stateMachine.doTransitions(); // enter initial state to prevent NPE

      mapGraphic.setDrawBoundingBox(true);
      mapGraphic.setDrawNormal(true);
      listToMergeGraphic.setDrawBoundingBox(true);
      listToMergeGraphic.setDrawNormal(true);

      getChildren().add(mapGraphic);
      getChildren().add(listToMergeGraphic);
      getChildren().add(meshGraphic);

      animationTimer.start();
   }

   private void onBoundingBoxCollisionEntry()
   {
      Map<PlanarRegion, List<PlanarRegion>> boundingBox3DCollisions = detectLocalBoundingBox3DCollisions(map,
                                                                                                         listToMerge,
                                                                                                         parameters.getBoundingBoxHeight());
      Map<PlanarRegion, List<PlanarRegion>> normalSimilarityFiltered = filterMatchesBasedOnNormalSimilarity(boundingBox3DCollisions, 0.9);
      matchesWithReferencePoints = filterMatchesBasedOn2DBoundingBoxShadow(0.01, normalSimilarityFiltered);

      meshGraphic.generateMeshesAsync();

      mapGraphic.generateMeshesAsync(map);
      listToMergeGraphic.generateMeshesAsync(listToMerge);
   }

   private void meshBuilder(JavaFXMeshBuilder meshBuilder)
   {
      for (PairList<PlanarRegion, Point2D> matchedReferencePoints : matchesWithReferencePoints.values())
      {
         for (ImmutablePair<PlanarRegion, Point2D> matchedReferencePoint : matchedReferencePoints)
         {
            Point3D matchedReferencePoint3D = new Point3D(matchedReferencePoint.getRight());
            RigidBodyTransform transformToWorld = PlanarRegionTools.getTransformToWorld(matchedReferencePoint.getLeft());
            matchedReferencePoint3D.applyTransform(transformToWorld);
            meshBuilder.addSphere(0.01, matchedReferencePoint3D);
         }
      }
   }

   private void fxUpdate(long now)
   {
      mapGraphic.update();
      listToMergeGraphic.update();
      meshGraphic.update();
   }

   public void copyDataIn(PlanarRegionsList map, PlanarRegionsList listToMerge)
   {
      this.map = map.copy();
      this.listToMerge = listToMerge.copy();
   }

   public void step()
   {
      nextStep.set();
      stateMachine.doActionAndTransition();
   }

   public SLAMVisualizationState getState()
   {
      return stateMachine.getCurrentStateKey();
   }

   public void setStateListener(Consumer<SLAMVisualizationState> stateListener)
   {
      this.stateListener = stateListener;
   }
}
