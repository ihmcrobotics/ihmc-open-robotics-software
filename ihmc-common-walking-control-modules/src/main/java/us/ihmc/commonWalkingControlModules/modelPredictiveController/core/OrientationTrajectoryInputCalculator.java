package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DirectOrientationValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationValueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.matrixlib.MatrixTools;

// TODO none of the methods in this class use the compressed block formulation
public class OrientationTrajectoryInputCalculator
{
   private final SE3MPCIndexHandler indexHandler;

   private static final DMatrixRMaj identity6 = CommonOps_DDRM.identity(6);

   public OrientationTrajectoryInputCalculator(SE3MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }


   public boolean compute(QPInputTypeA inputToPack, DirectOrientationValueCommand command)
   {
      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.setWeight(command.getObjectiveWeight());
      inputToPack.setUseWeightScalar(true);
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      int segmentNumber = command.getSegmentNumber();

      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);

      // V = This
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, orientationIndex, identity6, 0, 0, 6, 6, 1.0);
      inputToPack.getTaskObjective().set(command.getObjectiveValue());

      return true;
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

      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int numberOfLinearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);

      // V = A This + B c + C -> A This + B c = V - C
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, orientationIndex, command.getAMatrix(), 0, 0, 6, 6, 1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, comIndex, command.getBMatrix(), 0, 0, 6, numberOfLinearVariables, 1.0);

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

      int nextOrientationIndex = indexHandler.getOrientationStartIndex(segmentNumber + 1);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int numberOfLinearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);

      // Next = A This + B c + C -> Next - A This - B c = C
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, orientationIndex, command.getAMatrix(), 0, 0, 6, 6, -1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, comIndex, command.getBMatrix(), 0, 0, 6, numberOfLinearVariables, -1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, nextOrientationIndex, identity6, 0, 0, 6, 6, 1.0);

      inputToPack.getTaskObjective().set(command.getCMatrix());

      return true;
   }

   private final DMatrixRMaj orientationWeight = new DMatrixRMaj(6, 6);

   public boolean compute(int tick, QPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int numberOfLinearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);

      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
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

      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);

      // A This + B c + C = 0 -> A This + B c = -C
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, orientationIndex, command.getAMatrix(tick), 0, 0, 6, 6, 1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, comIndex, command.getBMatrix(tick), 0, 0, 6, numberOfLinearVariables, 1.0);
      CommonOps_DDRM.scale(-1.0, command.getCMatrix(tick), inputToPack.getTaskObjective());

      return true;
   }
}
