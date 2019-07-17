package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.scene.Group;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;

import java.util.function.Consumer;

import static us.ihmc.humanoidBehaviors.ui.slam.PlanarRegionSLAMGraphic.SLAMVisualizationState.*;

public class PlanarRegionSLAMGraphic extends Group
{
   private final StateMachine<SLAMVisualizationState, State> stateMachine;

   public enum SLAMVisualizationState
   {
      Hidden, BoundingBoxCollision
   }

   private PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::fxUpdate);
   private PlanarRegionsGraphic mapGraphic = new PlanarRegionsGraphic();
   private PlanarRegionsGraphic listToMergeGraphic = new PlanarRegionsGraphic();

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

      mapGraphic.setShowBoundingBox(true);
      listToMergeGraphic.setShowBoundingBox(true);

      getChildren().add(mapGraphic);
      getChildren().add(listToMergeGraphic);

      animationTimer.start();
   }

   private void onBoundingBoxCollisionEntry()
   {
      mapGraphic.generateMeshesAsync(map);
      listToMergeGraphic.generateMeshesAsync(listToMerge);
   }

   private void fxUpdate(long now)
   {
      mapGraphic.update();
      listToMergeGraphic.update();
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
