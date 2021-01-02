package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;

public class ContinuousMPCQPInputCalculator extends MPCQPInputCalculator
{
   private final ContinuousMPCIndexHandler indexHandler;

   private final OrientationDynamicsCommandCalculator orientationDynamicsCommandCalculator;

   public ContinuousMPCQPInputCalculator(ContinuousMPCIndexHandler indexHandler, double mass, double gravityZ)
   {
      super(indexHandler, gravityZ);
      this.indexHandler = indexHandler;
      this.orientationDynamicsCommandCalculator = new OrientationDynamicsCommandCalculator(indexHandler, mass);
   }

   @Override
   public boolean calculateContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      if (objective.getValueType() == MPCValueType.ORIENTATION)
      {
         return calculateOrientationContinuityObjective(inputToPack, objective);
      }

      return super.calculateContinuityObjective(inputToPack, objective);
   }

   public boolean calculateOrientationContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double weight = objective.getWeight();

      int firstYawStartIndex = indexHandler.getYawCoefficientsStartIndex(firstSegmentNumber);
      int firstPitchStartIndex = indexHandler.getPitchCoefficientsStartIndex(firstSegmentNumber);
      int firstRollStartIndex = indexHandler.getRollCoefficientsStartIndex(firstSegmentNumber);
      int secondYawStartIndex = indexHandler.getYawCoefficientsStartIndex(secondSegmentNumber);
      int secondPitchStartIndex = indexHandler.getPitchCoefficientsStartIndex(secondSegmentNumber);
      int secondRollStartIndex = indexHandler.getRollCoefficientsStartIndex(secondSegmentNumber);

      OrientationCoefficientJacobianCalculator.calculateAngularJacobian(firstYawStartIndex,
                                                                        firstPitchStartIndex,
                                                                        firstRollStartIndex,
                                                                        objective.getOmega(),
                                                                        firstSegmentDuration,
                                                                        inputToPack.getTaskJacobian(),
                                                                        objective.getDerivativeOrder(),
                                                                        1.0);
      OrientationCoefficientJacobianCalculator.calculateAngularJacobian(secondYawStartIndex,
                                                                        secondPitchStartIndex,
                                                                        secondRollStartIndex,
                                                                        objective.getOmega(),
                                                                        0.0,
                                                                        inputToPack.getTaskJacobian(),
                                                                        objective.getDerivativeOrder(),
                                                                        -1.0);

      inputToPack.getTaskObjective().zero();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   @Override
   public boolean calculateValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      if (objective.getValueType() == MPCValueType.ORIENTATION)
         return calculateOrientationValueObjective(inputToPack, objective);

      return super.calculateValueObjective(inputToPack, objective);
   }

   private boolean calculateOrientationValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double weight = objective.getWeight();

      int yawStartCol = indexHandler.getOrientationCoefficientsStartIndex(segmentNumber);
      int pitchStartCol = indexHandler.getOrientationCoefficientsStartIndex(segmentNumber);
      int rollStartCol = indexHandler.getOrientationCoefficientsStartIndex(segmentNumber);
      OrientationCoefficientJacobianCalculator.calculateAngularJacobian(yawStartCol,
                                                                        pitchStartCol,
                                                                        rollStartCol,
                                                                        objective.getOmega(),
                                                                        timeOfObjective,
                                                                        inputToPack.getTaskJacobian(),
                                                                        objective.getDerivativeOrder(),
                                                                        1.0);

      objective.getObjective().get(inputToPack.getTaskObjective());

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateOrientationTrackingObjective(QPInputTypeC inputToPack, OrientationTrackingCommand objective)
   {
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      if (!AngleTrackingCostCalculator.calculateTrackingObjective(inputToPack.getDirectCostHessian(),
                                                                  inputToPack.getDirectCostGradient(),
                                                                  objective.getYawTrackingCommand()))
         return false;
      if (!AngleTrackingCostCalculator.calculateTrackingObjective(inputToPack.getDirectCostHessian(),
                                                                  inputToPack.getDirectCostGradient(),
                                                                  objective.getPitchTrackingCommand()))
         return false;
      if (!AngleTrackingCostCalculator.calculateTrackingObjective(inputToPack.getDirectCostHessian(),
                                                                  inputToPack.getDirectCostGradient(),
                                                                  objective.getRollTrackingCommand()))
         return false;

      double weight = objective.getWeight();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateAngleTrackingCommand(QPInputTypeC inputToPack, AngleTrackingCommand objective)
   {
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      if (!AngleTrackingCostCalculator.calculateTrackingObjective(inputToPack.getDirectCostHessian(), inputToPack.getDirectCostGradient(), objective))
         return false;

      double weight = objective.getWeight();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateOrientationDynamicsObjective(QPInputTypeA inputToPack, OrientationDynamicsCommand command)
   {
      if (command.getNumberOfContacts() < 1)
         return false;

      int problemSize = 3;

      inputToPack.reshape(problemSize);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(command.getConstraintType());

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(command.getWeight());

      orientationDynamicsCommandCalculator.compute(command);
      inputToPack.getTaskJacobian().set(orientationDynamicsCommandCalculator.getOrientationJacobian());
      CommonOps_DDRM.addEquals(inputToPack.getTaskJacobian(), -1.0, orientationDynamicsCommandCalculator.getTorqueJacobian());

      return true;
   }
}
