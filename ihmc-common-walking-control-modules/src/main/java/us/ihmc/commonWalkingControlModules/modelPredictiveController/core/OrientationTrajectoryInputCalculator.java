package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationValueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.matrixlib.MatrixTools;

public class OrientationTrajectoryInputCalculator
{
   private final ImplicitSE3MPCIndexHandler indexHandler;

   private static final DMatrixRMaj identity6 = CommonOps_DDRM.identity(6);

   public OrientationTrajectoryInputCalculator(ImplicitSE3MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   public boolean compute(QPInputTypeA inputToPack, OrientationValueCommand command)
   {
      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.setWeight(command.getObjectiveWeight());
      inputToPack.setUseWeightScalar(true);
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      int segmentNumber = command.getSegmentNumber();

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationStartIndex(segmentNumber), command.getAMatrix(), 0, 0, 6, 6, 1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 indexHandler.getComCoefficientStartIndex(segmentNumber),
                                 command.getBMatrix(),
                                 0,
                                 0,
                                 6,
                                 LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber),
                                 1.0);

      CommonOps_DDRM.subtract(command.getObjectiveValue(), command.getCMatrix(), inputToPack.getTaskObjective());

      return true;
   }

   public boolean compute(QPInputTypeA inputToPack, OrientationContinuityCommand command)
   {
      int segmentNumber = command.getSegmentNumber();
      if (segmentNumber == indexHandler.getNumberOfSegments() - 1)
         return false;

      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.setWeight(command.getObjectiveWeight());
      inputToPack.setUseWeightScalar(true);

      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 indexHandler.getComCoefficientStartIndex(segmentNumber),
                                 command.getBMatrix(),
                                 0,
                                 0,
                                 6,
                                 LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber),
                                 -1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationStartIndex(segmentNumber + 1), identity6, 0, 0, 6, 6, 1.0);

      inputToPack.getTaskObjective().set(command.getCMatrix());

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 indexHandler.getOrientationStartIndex(segmentNumber),
                                 command.getAMatrix(),
                                 0,
                                 0,
                                 6,
                                 6,
                                 -1.0);

      return true;
   }

   private final DMatrixRMaj orientationWeight = new DMatrixRMaj(6, 6);

   public boolean compute(int tick, QPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int linearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int orientationVariables = segmentNumber > 0 ? ImplicitSE3MPCIndexHandler.variablesPerOrientationTick : 0;
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      inputToPack.setNumberOfVariables(linearVariables + orientationVariables);
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setUseWeightScalar(false);
      for (int i = 0; i < 3; i++)
      {
         orientationWeight.set(i, i, command.getAngleErrorMinimizationWeight());
         orientationWeight.set(i + 3, i + 3, command.getVelocityErrorMinimizationWeight());
      }
      inputToPack.setTaskWeightMatrix(orientationWeight);

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, 0, command.getBMatrix(tick), 0, 0, 6, linearVariables, 1.0);
      CommonOps_DDRM.scale(-1.0, command.getCMatrix(tick), inputToPack.getTaskObjective());

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 indexHandler.getOrientationStartIndex(segmentNumber),
                                 command.getAMatrix(tick),
                                 0,
                                 0,
                                 6,
                                 6,
                                 1.0);

      return true;
   }
}
