package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.alg.dense.mult.VectorVectorMult;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class FootStateTransitionCondition implements StateTransitionCondition
{
   private static final int velocitySignForSingularityEscape = 1;

   private final ConstraintType stateEnum;
   private final GeometricJacobian jacobian;
   private final NullspaceCalculator nullspaceCalculator;
   private final DenseMatrix64F jointVelocities;
   private final BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState;
   private final FootControlHelper footControlHelper;

   public FootStateTransitionCondition(AbstractFootControlState stateToTransitionTo, FootControlHelper footControlHelper,
         BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState)
   {
      this.stateEnum = stateToTransitionTo.getStateEnum();
      this.footControlHelper = footControlHelper;

      this.jacobian = footControlHelper.getJacobian();
      this.waitSingularityEscapeBeforeTransitionToNextState = waitSingularityEscapeBeforeTransitionToNextState;

      nullspaceCalculator = new NullspaceCalculator(jacobian.getNumberOfColumns(), true);
      jointVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jacobian.getJointsInOrder()), 1);
   }

   public boolean checkCondition()
   {
      boolean transitionRequested = footControlHelper.getRequestedState() == stateEnum;

      if (!transitionRequested)
         return false;

      boolean singularityEscapeDone = true;

      if (footControlHelper.isDoingSingularityEscape() && waitSingularityEscapeBeforeTransitionToNextState.getBooleanValue())
      {
         nullspaceCalculator.setMatrix(jacobian.getJacobianMatrix(), 1);
         DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();
         ScrewTools.packJointVelocitiesMatrix(jacobian.getJointsInOrder(), jointVelocities);
         double nullspaceVelocityDotProduct = VectorVectorMult.innerProd(nullspace, jointVelocities);

         int velocitySign = (int) Math.round(Math.signum(nullspaceVelocityDotProduct));
         boolean velocitySignOK = velocitySign == velocitySignForSingularityEscape;
         if (footControlHelper.isJacobianDeterminantInRange() || !velocitySignOK)
         {
            singularityEscapeDone = false;
         }
      }

      return singularityEscapeDone;
   }
}
