package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.matrixlib.MatrixTools;

public class OrientationTrajectoryInputCalculator
{
   private final OtherSE3MPCIndexHandler indexHandler;

   private static final DMatrixRMaj identity6 = CommonOps_DDRM.identity(6);

   public OrientationTrajectoryInputCalculator(OtherSE3MPCIndexHandler indexHandler, double mass, double gravity)
   {
      this.indexHandler = indexHandler;
   }

   public boolean computeConstraintForNextSegmentStart(QPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();
      if (segmentNumber == indexHandler.getNumberOfSegments() - 1)
         return false;

      inputToPack.setConstraintType(ConstraintType.EQUALITY);
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 indexHandler.getComCoefficientStartIndex(segmentNumber),
                                 command.getLastBMatrix(),
                                 0,
                                 0,
                                 6,
                                 LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber),
                                 -1.0);
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationStartIndices(segmentNumber + 1), identity6, 0, 0, 6, 6, 1.0);

      inputToPack.getTaskObjective().set(command.getLastCMatrix());

      if (segmentNumber == 0)
      {
         CommonOps_DDRM.multAdd(command.getLastAMatrix(), command.getInitialError(), inputToPack.getTaskObjective());
      }
      else
      {
         MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                    0,
                                    indexHandler.getOrientationStartIndices(segmentNumber),
                                    command.getLastAMatrix(),
                                    0,
                                    0,
                                    6,
                                    6,
                                    -1.0);
      }

      return true;
   }

   public boolean computeErrorMinimizationCommand(int tick, QPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int linearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int orientationVariables = segmentNumber > 0 ? OtherSE3MPCIndexHandler.variablesPerOrientationTick : 0;
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      inputToPack.setNumberOfVariables(linearVariables + orientationVariables);
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, 0, command.getBMatrix(tick), 0, 0, 6, linearVariables, 1.0);
      CommonOps_DDRM.scale(-1.0, command.getCMatrix(tick), inputToPack.getTaskObjective());

      if (segmentNumber == 0)
      {
         CommonOps_DDRM.multAdd(-1.0, command.getAMatrix(tick), command.getInitialError(), inputToPack.getTaskObjective());
      }
      else
      {
         MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(),
                                    0,
                                    indexHandler.getOrientationStartIndices(segmentNumber),
                                    command.getAMatrix(tick),
                                    0,
                                    0,
                                    6,
                                    6,
                                    1.0);
      }

      return true;
   }
}
