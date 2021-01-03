package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.AngleTrackingCostCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.OrientationCoefficientJacobianCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.OrientationDynamicsCommandCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;

public class DiscreteMPCQPInputCalculator extends MPCQPInputCalculator
{
   private final DiscreteMPCIndexHandler indexHandler;
   private final DiscreteOrientationDynamicsCommandCalculator dynamicsCommandCalculator;

   public DiscreteMPCQPInputCalculator(DiscreteMPCIndexHandler indexHandler, double mass, double gravityZ)
   {
      super(indexHandler, gravityZ);
      this.indexHandler = indexHandler;
      dynamicsCommandCalculator = new DiscreteOrientationDynamicsCommandCalculator(indexHandler, mass);
   }


   public boolean calculateOrientationDynamicsObjective(QPInputTypeA inputToPack, DiscreteOrientationDynamicsCommand command)
   {
      if (command.getNumberOfSegments() < 1)
         return false;

      int problemSize = 3;

      inputToPack.reshape(problemSize);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(command.getConstraintType());

      dynamicsCommandCalculator.compute(command);

      // TODO finish computing things

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(command.getWeight());


      return true;
   }
}
