package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class ProximityConditionExecutor
{
   private final BehaviorTreeSceneState scene;
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalDouble distance;
   private final CRDTBidirectionalDouble maxDistanceToObject;
   private double conditionStartTime = -1.0;

   public ProximityConditionExecutor(ConditionNodeState state, BehaviorTreeSceneState scene)
   {
      this.state = state;
      this.scene = scene;

      definition = state.getDefinition();

      distance = state.getProximityCheck().getCurrentDistance();
      maxDistanceToObject = definition.getProximityCheck().getCRDTMaxDistanceToObject();
   }

   public void update()
   {
      boolean canExecute = true;
      canExecute &= scene.findFrameByName(definition.getProximityCheck().getReferenceFrameName()) != null;

      boolean manageMissingFrameInternally = definition.getProximityCheck().getCRDTManageMissingFrameInternally().getValue();
      boolean missingFrame = scene.findFrameByName(definition.getProximityCheck().getObjectFrameName()) == null;
      if (!manageMissingFrameInternally)
      {
         canExecute &= !missingFrame;
      }
      state.setCanExecute(canExecute);
      state.getProximityCheck().setMissingFrame(missingFrame);

      if (!missingFrame)
      {
         if ((state.getIsExecuting() || state.isEvaluatingCondition()) && state.getCanExecute())
         {
            FramePose3D objectFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                          scene.findFrameByName(definition.getProximityCheck().getObjectFrameName())
                                                                               .getTransformToWorldFrame());
            FramePose3D referenceFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                             scene.findFrameByName(definition.getProximityCheck().getReferenceFrameName())
                                                                                  .getTransformToWorldFrame());
            switch (definition.getProximityCheck().getType().getValue())
            {
               case XYZ -> distance.setValue(objectFramePose.getPosition().distance(referenceFramePose.getPosition()));
               case XY -> distance.setValue(objectFramePose.getPosition().distanceXY(referenceFramePose.getPosition()));
               case Z -> distance.setValue(Math.abs(objectFramePose.getPosition().getZ() - referenceFramePose.getPosition().getZ()));
            }
         }
      }

   }

   public void updateCurrentlyExecuting()
   {
      if (!state.getProximityCheck().getMissingFrame().getValue())
      {
         if (!state.isEvaluatingCondition())
         {
            conditionStartTime = System.nanoTime();
            state.setEvaluatingConditionValue(true);
         }

         boolean conditionValue = distance.getValue() >= 0 && distance.getValue() < maxDistanceToObject.getValue();
         state.setConditionValue(conditionValue);

         if (!conditionValue)
         {
            if (getTimeElapsedInCondition() > definition.getProximityCheck().getCRDTMaxEvaluationTime().getValue())
            {
               state.setFailed(true);
               state.setEvaluatingConditionValue(false);
            }
         }
      }

      state.setIsExecuting(false); // Completes immediately
   }

   private double getTimeElapsedInCondition()
   {
      if (conditionStartTime < 0.0)
         return 0.0;
      else
         return (System.nanoTime() - conditionStartTime) * 1.0e-9;
   }
}