package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DirectOrientationValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationValueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;

public class OrientationTrajectoryInputCalculator
{
   private final SE3MPCIndexHandler indexHandler;

   private static final DMatrixRMaj identity6 = CommonOps_DDRM.identity(6);

   private final DMatrixRMaj tempObjective = new DMatrixRMaj(0, 0);

   public OrientationTrajectoryInputCalculator(SE3MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   public int compute(NativeQPInputTypeA inputToPack, DirectOrientationValueCommand command)
   {
      int orientationIndex = indexHandler.getOrientationStartIndex(command.getSegmentNumber());

      if (computeInternal(inputToPack, command, indexHandler.getTotalProblemSize(), orientationIndex))
         return 0;
      else
         return -1;
   }

   public int computeCompact(NativeQPInputTypeA inputToPack, DirectOrientationValueCommand command)
   {
      int orientationIndex = indexHandler.getOrientationStartIndex(command.getSegmentNumber());

      if (computeInternal(inputToPack, command, SE3MPCIndexHandler.variablesPerOrientationTick, 0))
         return orientationIndex;
      else
         return -1;
   }

   private boolean computeInternal(NativeQPInputTypeA inputToPack, DirectOrientationValueCommand command, int numberOfVariables, int startIndex)
   {
      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.setWeight(command.getObjectiveWeight());
      inputToPack.setUseWeightScalar(true);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(SE3MPCIndexHandler.variablesPerOrientationTick);

      inputToPack.getTaskJacobian().zero();

      // V = This
      inputToPack.getTaskJacobian().insert(identity6, 0, startIndex);
      inputToPack.getTaskObjective().set(command.getObjectiveValue());

      return true;
   }

   public int compute(NativeQPInputTypeA inputToPack, OrientationValueCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);

      if (computeInternal(inputToPack, command, indexHandler.getTotalProblemSize(), comIndex, orientationIndex))
         return 0;
      else
         return -1;
   }

   public int computeCompact(NativeQPInputTypeA inputToPack, OrientationValueCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);

      if (computeInternal(inputToPack, command, indexHandler.getVariablesInSegment(segmentNumber), SE3MPCIndexHandler.variablesPerOrientationTick, 0))
         return orientationIndex;
      else
         return -1;
   }

   private boolean computeInternal(NativeQPInputTypeA inputToPack, OrientationValueCommand command, int numberOfVariables, int comIndex, int orientationIndex)
   {
      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.setWeight(command.getObjectiveWeight());
      inputToPack.setUseWeightScalar(command.useWeightScalar());
      inputToPack.setTaskWeightMatrix(command.getWeightMatrix());
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(SE3MPCIndexHandler.variablesPerOrientationTick);

      tempObjective.reshape(SE3MPCIndexHandler.variablesPerOrientationTick, 1);

      // V = A This + B c + C -> A This + B c = V - C
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskJacobian().insert(command.getAMatrix(), 0, orientationIndex);
      inputToPack.getTaskJacobian().insert(command.getBMatrix(), 0, comIndex);

      CommonOps_DDRM.subtract(command.getObjectiveValue(), command.getCMatrix(), tempObjective);
      inputToPack.getTaskObjective().set(tempObjective);

      return true;
   }

   public int compute(NativeQPInputTypeA inputToPack, OrientationContinuityCommand command)
   {
      int segmentNumber = command.getSegmentNumber();
      if (segmentNumber == indexHandler.getNumberOfSegments() - 1)
         return -1;

      if (computeInternal(inputToPack,
                          command,
                          indexHandler.getTotalProblemSize(),
                          indexHandler.getComCoefficientStartIndex(segmentNumber),
                          indexHandler.getOrientationStartIndex(segmentNumber),
                          indexHandler.getOrientationStartIndex(segmentNumber + 1)))
         return 0;
      else
         return -1;
   }

   public int computeCompact(NativeQPInputTypeA inputToPack, OrientationContinuityCommand command)
   {
      int segmentNumber = command.getSegmentNumber();
      if (segmentNumber == indexHandler.getNumberOfSegments() - 1)
         return -1;

      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber) - orientationIndex;
      int nextOrientationIndex = indexHandler.getOrientationStartIndex(segmentNumber + 1) - orientationIndex;
      int numberOfVariables = indexHandler.getVariablesInSegment(segmentNumber) + SE3MPCIndexHandler.variablesPerOrientationTick;

      if (computeInternal(inputToPack, command, numberOfVariables, comIndex, 0, nextOrientationIndex))
         return orientationIndex;
      else
         return -1;
   }

   private boolean computeInternal(NativeQPInputTypeA inputToPack,
                                   OrientationContinuityCommand command,
                                   int numberOfVariables,
                                   int comIndex,
                                   int orientationIndex,
                                   int nextOrientationIndex)
   {
      int segmentNumber = command.getSegmentNumber();
      if (segmentNumber == indexHandler.getNumberOfSegments() - 1)
         return false;

      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.setWeight(command.getObjectiveWeight());
      inputToPack.setUseWeightScalar(true);

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(SE3MPCIndexHandler.variablesPerOrientationTick);

      // Next = A This + B c + C -> Next - A This - B c = C
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskJacobian().insertScaled(command.getAMatrix(), 0, orientationIndex, -1.0);
      inputToPack.getTaskJacobian().insertScaled(command.getBMatrix(), 0, comIndex, -1.0);
      inputToPack.getTaskJacobian().insert(identity6, 0, nextOrientationIndex);

      inputToPack.getTaskObjective().set(command.getCMatrix());

      return true;
   }

   public int compute(int tick, NativeQPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);

      if (computeInternal(tick, inputToPack, command, indexHandler.getTotalProblemSize(), comIndex, orientationIndex))
         return 0;
      else
         return -1;
   }

   public int computeCompact(int tick, NativeQPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int numberOfVariables = indexHandler.getVariablesInSegment(segmentNumber);

      if (computeInternal(tick, inputToPack, command, numberOfVariables, comIndex - orientationIndex, 0))
         return orientationIndex;
      else
         return -1;
   }

   private final DMatrixRMaj orientationWeight = new DMatrixRMaj(6, 6);

   public boolean computeInternal(int tick,
                                  NativeQPInputTypeA inputToPack,
                                  OrientationTrajectoryCommand command,
                                  int numberOfVariables,
                                  int comIndex,
                                  int orientationIndex)
   {
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(SE3MPCIndexHandler.variablesPerOrientationTick);

      inputToPack.getTaskObjective().zero();
      inputToPack.setUseWeightScalar(false);
      for (int i = 0; i < 3; i++)
      {
         orientationWeight.set(i, i, command.getAngleErrorMinimizationWeight());
         orientationWeight.set(i + 3, i + 3, command.getVelocityErrorMinimizationWeight());
      }
      inputToPack.setTaskWeightMatrix(orientationWeight);

      // A This + B c + C = 0 -> A This + B c = -C
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskJacobian().insert(command.getAMatrix(tick), 0, orientationIndex);
      inputToPack.getTaskJacobian().insert(command.getBMatrix(tick), 0, comIndex);

      inputToPack.getTaskObjective().scale(-1.0, command.getCMatrix(tick));

      return true;
   }

   public int computeAngleErrorMinimization(int tick, NativeQPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);

      if (computeAngleErrorMinimizationInternal(tick, inputToPack, command, indexHandler.getTotalProblemSize(), comIndex, orientationIndex))
         return 0;
      else
         return -1;
   }

   public int computeAngleErrorMinimizationCompact(int tick, NativeQPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int numberOfVariables = indexHandler.getVariablesInSegment(segmentNumber);

      if (computeAngleErrorMinimizationInternal(tick, inputToPack, command, numberOfVariables, comIndex - orientationIndex, 0))
         return orientationIndex;
      else
         return -1;
   }

   public boolean computeAngleErrorMinimizationInternal(int tick,
                                                        NativeQPInputTypeA inputToPack,
                                                        OrientationTrajectoryCommand command,
                                                        int numberOfVariables,
                                                        int comIndex,
                                                        int orientationIndex)
   {
      int segmentNumber = command.getSegmentNumber();

      int numberOfLinearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);

      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);

      inputToPack.getTaskJacobian().zero();
      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(command.getAngleErrorMinimizationWeight());

      // A This + B c + C = 0 -> A This + B c = -C
      inputToPack.getTaskJacobian().insert(command.getAMatrix(tick), 0, 3, 0, 6, 0, orientationIndex);
      inputToPack.getTaskJacobian().insert(command.getBMatrix(tick), 0, 3, 0, numberOfLinearVariables, 0, comIndex);

      inputToPack.getTaskObjective().insertScaled(command.getCMatrix(tick), 0, 3, 0, 1, 0, 0, -1.0);

      return true;
   }

   public int computeVelocityErrorMinimization(int tick, NativeQPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);

      if (computeVelocityErrorMinimizationInternal(tick, inputToPack, command, indexHandler.getTotalProblemSize(), comIndex, orientationIndex))
         return 0;
      else
         return -1;
   }

   public int computeVelocityErrorMinimizationCompact(int tick, NativeQPInputTypeA inputToPack, OrientationTrajectoryCommand command)
   {
      int segmentNumber = command.getSegmentNumber();

      int comIndex = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int orientationIndex = indexHandler.getOrientationStartIndex(segmentNumber);
      int numberOfVariables = indexHandler.getVariablesInSegment(segmentNumber);

      if (computeVelocityErrorMinimizationInternal(tick, inputToPack, command, numberOfVariables, comIndex - orientationIndex, 0))
         return orientationIndex;
      else
         return -1;
   }

   public boolean computeVelocityErrorMinimizationInternal(int tick,
                                                           NativeQPInputTypeA inputToPack,
                                                        OrientationTrajectoryCommand command,
                                                        int numberOfVariables,
                                                        int comIndex,
                                                        int orientationIndex)
   {
      int segmentNumber = command.getSegmentNumber();

      int numberOfLinearVariables = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);

      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);

      inputToPack.getTaskJacobian().zero();
      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(command.getVelocityErrorMinimizationWeight());

      // A This + B c + C = 0 -> A This + B c = -C
      inputToPack.getTaskJacobian().insert(command.getAMatrix(tick), 3, 6, 0, 6, 0, orientationIndex);
      inputToPack.getTaskJacobian().insert(command.getBMatrix(tick), 3, 6, 0, numberOfLinearVariables, 0, comIndex);

      inputToPack.getTaskObjective().insertScaled(command.getCMatrix(tick), 3, 6, 0, 1, 0, 0, -1.0);

      return true;
   }
}
