package us.ihmc.behaviors.logic.condition;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class ProximityConditionExecutor
{
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalDouble distance;
   private final CRDTBidirectionalDouble maxDistanceToObject;
   private double conditionStartTime = -1.0;

   public ProximityConditionExecutor(ConditionNodeState state, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.state = state;
      this.referenceFrameLibrary = referenceFrameLibrary;

      definition = state.getDefinition();

      distance = state.getProximityCheck().getCurrentDistance();
      maxDistanceToObject = definition.getProximityCheck().getCRDTMaxDistanceToObject();
   }

   public void update()
   {
      boolean canExecute = true;
      canExecute &= referenceFrameLibrary.findFrameByName(definition.getProximityCheck().getObjectFrameName()) != null;
      canExecute &= referenceFrameLibrary.findFrameByName(definition.getProximityCheck().getReferenceFrameName()) != null;
      state.setCanExecute(canExecute);

      if ((state.getIsExecuting() || state.getIsNextForExecution() || state.isEvaluatingCondition()) && state.getCanExecute())
      {
         FramePose3D objectFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                       referenceFrameLibrary.findFrameByName(definition.getProximityCheck().getObjectFrameName())
                                                                            .getTransformToWorldFrame());
         FramePose3D referenceFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                          referenceFrameLibrary.findFrameByName(definition.getProximityCheck().getReferenceFrameName())
                                                                               .getTransformToWorldFrame());
         switch (definition.getProximityCheck().getType().getValue())
         {
            case XYZ -> distance.setValue(objectFramePose.getPosition().distance(referenceFramePose.getPosition()));
            case XY -> distance.setValue(objectFramePose.getPosition().distanceXY(referenceFramePose.getPosition()));
            case Z -> distance.setValue(Math.abs(objectFramePose.getPosition().getZ() - referenceFramePose.getPosition().getZ()));
         }
      }
   }

   public void updateCurrentlyExecuting()
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
