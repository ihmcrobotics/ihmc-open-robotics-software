package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.tools.Timer;

public class ProximityConditionExecutor
{
   private final BehaviorTreeSceneState scene;
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;
   private final ProximityConditionState proximityState;
   private final ProximityConditionDefinition proximityDefinition;

   private ReferenceFrame frameA;
   private ReferenceFrame frameB;
   private final FramePoint3D frameAPoint = new FramePoint3D();
   private final FramePoint3D frameBPoint = new FramePoint3D();
   private final Timer timer = new Timer();
   private final Vector3D bToA = new Vector3D();

   public ProximityConditionExecutor(ConditionNodeState state, BehaviorTreeSceneState scene)
   {
      this.scene = scene;
      this.state = state;
      definition = state.getDefinition();
      proximityState = state.getProximityCheck();
      proximityDefinition = definition.getProximityCheck();
   }

   public void update()
   {
      frameA = scene.findFrameByName(proximityDefinition.getFrameNameA());
      frameB = scene.findFrameByName(proximityDefinition.getFrameNameB());
      proximityState.setFrameAIsPresent(frameA != null);
      proximityState.setFrameBIsPresent(frameB != null);
      boolean canExecute = frameA != null && frameB != null;
      state.setCanExecute(canExecute);

      if (canExecute)
      {
         frameAPoint.setFromReferenceFrame(frameA);
         frameBPoint.setFromReferenceFrame(frameB);
         bToA.sub(frameBPoint, frameAPoint);
         proximityState.setVectorBToA(bToA, 1e-3);
      }
   }

   public void triggerExecution()
   {
      timer.reset();
   }

   public void updateCurrentlyExecuting()
   {
      if (frameA == null)
         state.getLogger().error("Frame A is null.");
      if (frameB == null)
         state.getLogger().error("Frame B is null.");

      double distance = Double.NaN;
      if (frameA != null && frameB != null)
      {
         distance = switch (proximityDefinition.getDistanceType())
         {
            case XYZ -> bToA.norm();
            case XY -> Math.hypot(bToA.getX(), bToA.getY());
            case Z -> Math.abs(bToA.getZ());
         };
      }

      boolean timeout = !timer.isRunning(proximityDefinition.getTimeout());
      if (timeout)
      {
         state.getLogger().error("Timeout after %.2f seconds.".formatted(proximityDefinition.getTimeout()));

         if (frameA != null && frameB != null)
         {
            state.getLogger().error("Distance not in range: %.2f < %.2f < %.2f"
                                     .formatted(proximityDefinition.getMinDistance(), distance, proximityDefinition.getMaxDistance()));
         }
      }

      if (frameA == null || frameB == null || timeout)
      {
         state.setFailed(true);
         state.setIsExecuting(false);
         return;
      }

      if (distance > proximityDefinition.getMinDistance() && distance < proximityDefinition.getMaxDistance())
         state.setIsExecuting(false);
   }
}
