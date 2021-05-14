package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.matrixlib.MatrixTools;

public class OrientationInputCalculator
{
   private final SE3MPCIndexHandler indexHandler;

   final OrientationDynamicsCalculator dynamicsCalculator;

   private static final DMatrixRMaj identity6 = CommonOps_DDRM.identity(6);

   public OrientationInputCalculator(SE3MPCIndexHandler indexHandler, double mass, double gravity)
   {
      dynamicsCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravity);
      this.indexHandler = indexHandler;
   }

   public boolean compute(QPInputTypeA inputToPack, DiscreteAngularVelocityOrientationCommand command)
   {
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(ConstraintType.EQUALITY);

      dynamicsCalculator.compute(command);

      int coefficientsInSegment = computeCoefficientsInSegment(command.getSegmentNumber());

      if (command.getEndDiscreteTickId() == 0)
         setUpConstraintForFirstTick(inputToPack, command, coefficientsInSegment);
      else
         setUpConstraintForRegularTick(inputToPack, command, coefficientsInSegment);

      return true;
   }

   DiscretizationCalculator getDiscretizationCalculator()
   {
      return dynamicsCalculator.getDiscretizationCalculator();
   }

   DMatrixRMaj getContinuousAMatrix()
   {
      return dynamicsCalculator.getContinuousAMatrix();
   }

   DMatrixRMaj getContinuousBMatrix()
   {
      return dynamicsCalculator.getContinuousBMatrix();
   }

   DMatrixRMaj getContinuousCMatrix()
   {
      return dynamicsCalculator.getContinuousCMatrix();
   }

   DMatrixRMaj getDiscreteAMatrix()
   {
      return dynamicsCalculator.getDiscreteAMatrix();
   }

   DMatrixRMaj getDiscreteBMatrix()
   {
      return dynamicsCalculator.getDiscreteBMatrix();
   }

   DMatrixRMaj getDiscreteCMatrix()
   {
      return dynamicsCalculator.getDiscreteCMatrix();
   }

   DMatrixRMaj getB0()
   {
      return dynamicsCalculator.getB0();
   }

   DMatrixRMaj getB1()
   {
      return dynamicsCalculator.getB1();
   }

   DMatrixRMaj getB2()
   {
      return dynamicsCalculator.getB2();
   }

   DMatrixRMaj getB3()
   {
      return dynamicsCalculator.getB3();
   }

   DMatrixRMaj getB4()
   {
      return dynamicsCalculator.getB4();
   }

   private int computeCoefficientsInSegment(int segmentNumber)
   {
      return LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
   }


   private final DMatrixRMaj initialStateVector = new DMatrixRMaj(6, 1);

   private void setUpConstraintForFirstTick(QPInputTypeA inputToPack,
                                            DiscreteAngularVelocityOrientationCommand command,
                                            int coefficientsInSegment)
   {
      command.getCurrentAxisAngleError().get(initialStateVector);
      command.getCurrentBodyAngularVelocityError().get(3, initialStateVector);

      CommonOps_DDRM.mult(dynamicsCalculator.getDiscreteAMatrix(), initialStateVector, inputToPack.getTaskObjective());
      CommonOps_DDRM.addEquals(inputToPack.getTaskObjective(), dynamicsCalculator.getDiscreteCMatrix());

      int startCoMIndex = indexHandler.getComCoefficientStartIndex(command.getSegmentNumber());
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, startCoMIndex, dynamicsCalculator.getDiscreteBMatrix(), 0, 0, 6, coefficientsInSegment, -1.0);

      // FIXME could likely do a "set" instead of "add"
      int orientationIndex = indexHandler.getOrientationTickStartIndex(command.getEndDiscreteTickId());
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, orientationIndex, identity6, 0, 0, 6, 6, 1.0);
   }

   private void setUpConstraintForRegularTick(QPInputTypeA inputToPack,
                                              DiscreteAngularVelocityOrientationCommand command,
                                              int coefficientsInSegment)
   {
      inputToPack.getTaskObjective().set(dynamicsCalculator.getDiscreteCMatrix());

      int startCoMIndex = indexHandler.getComCoefficientStartIndex(command.getSegmentNumber());
      MatrixTools.setMatrixBlock(inputToPack.getTaskJacobian(), 0, startCoMIndex, dynamicsCalculator.getDiscreteBMatrix(), 0, 0, 6, coefficientsInSegment, -1.0);

      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationTickStartIndex(command.getEndDiscreteTickId()),
                                 identity6, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationTickStartIndex(command.getEndDiscreteTickId() - 1), dynamicsCalculator.getDiscreteAMatrix(), 0, 0, 6, 6, -1.0);
   }
}
