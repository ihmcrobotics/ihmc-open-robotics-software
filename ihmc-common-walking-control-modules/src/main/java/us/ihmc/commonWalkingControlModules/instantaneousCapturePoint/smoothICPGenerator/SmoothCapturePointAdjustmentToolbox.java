package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class SmoothCapturePointAdjustmentToolbox
{
   private static final int defaultSize = 100;

   private final DenseMatrix64F generalizedAlphaPrimeRowSegment1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeRowSegment1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedGammaPrimeMatrixSegment1 = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1 = new DenseMatrix64F(1, defaultSize);

   private final DenseMatrix64F alphaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F betaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F gammaPrimeMatrixSegment2 = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F alphaBetaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);

   private final DenseMatrix64F boundaryConditionMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F boundaryConditionMatrixInverse = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F boundaryConditionVector = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F polynomialCoefficientCombinedVectorAdjustment = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F polynomialCoefficientVectorAdjustmentSegment1 = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F polynomialCoefficientVectorAdjustmentSegment2 = new DenseMatrix64F(defaultSize, 1);

   private final LinearSolver<DenseMatrix64F> pseudoInverseSolver = new SolvePseudoInverseSvd();

   private List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList = new ArrayList<FrameTuple3D<?, ?>>();
   private FrameTuple3D<?, ?> icpQuantityInitialSegment1 = new FramePoint3D();

   private final SmoothCapturePointToolbox icpToolbox;

   public SmoothCapturePointAdjustmentToolbox(SmoothCapturePointToolbox smoothCapturePointToolbox)
   {
      this.icpToolbox = smoothCapturePointToolbox;

      icpQuantityInitialConditionList.add(new FramePoint3D());
      while (icpQuantityInitialConditionList.size() < defaultSize)
      {
         icpQuantityInitialConditionList.add(new FrameVector3D());
      }
   }

   public void setICPInitialConditions(double localTime, List<FramePoint3D> exitCornerPointsFromCoPs, List<FrameTrajectory3D> copPolynomials3D,
                                       int currentSwingSegment, boolean isInitialTransfer, double omega0)
   {
      if (currentSwingSegment < 0)
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(0);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            copPolynomial3D.getDerivative(i, localTime, icpQuantityInitialCondition);
         }
      }
      else
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(currentSwingSegment);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, localTime, i, copPolynomial3D,
                                                                            exitCornerPointsFromCoPs.get(currentSwingSegment), icpQuantityInitialCondition);
         }
      }
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing(List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack,
                                                            List<FrameTrajectory3D> copPolynomials3D, double omega0)
   {
      adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, copPolynomials3D, icpQuantityInitialConditionList, entryCornerPointsToPack,
                                                     exitCornerPointsToPack);
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing3D(double omega0, List<FrameTrajectory3D> copPolynomials3D,
                                                              List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList,
                                                              List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack)
   {
      FrameTrajectory3D cmpPolynomial3DSegment1 = copPolynomials3D.get(0);
      FrameTrajectory3D cmpPolynomial3DSegment2 = copPolynomials3D.get(1);
      FramePoint3D icpPositionFinalSegment2 = exitCornerPointsToPack.get(1);
      for (Direction direction : Direction.values())
      {
         Trajectory cmpPolynomialSegment1 = cmpPolynomial3DSegment1.getTrajectory(direction);
         Trajectory cmpPolynomialSegment2 = cmpPolynomial3DSegment2.getTrajectory(direction);

         double icpPositionFinalSegment2Scalar = icpPositionFinalSegment2.getElement(direction.getIndex());

         int numberOfCoefficients = cmpPolynomialSegment1.getNumberOfCoefficients();
         int numberOfConstrainedDerivatives = cmpPolynomialSegment1.getNumberOfCoefficients() / 2;

         initializeMatrices1D(numberOfCoefficients, numberOfConstrainedDerivatives);

         populateBoundaryConditionMatrices1D(omega0, direction, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1,
                                             cmpPolynomialSegment2, icpQuantityInitialConditionList, icpPositionFinalSegment2Scalar);
         computeAdjustedPolynomialCoefficientVectors1D(numberOfCoefficients);
         adjustCMPPolynomials(cmpPolynomialSegment1, cmpPolynomialSegment2);
      }
      icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsToPack, exitCornerPointsToPack, copPolynomials3D, omega0);
   }

   private void adjustCMPPolynomials(Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2)
   {
      cmpPolynomialSegment1.setDirectly(polynomialCoefficientVectorAdjustmentSegment1);
      cmpPolynomialSegment2.setDirectly(polynomialCoefficientVectorAdjustmentSegment2);
   }

   private void populateBoundaryConditionMatrices1D(double omega0, Direction direction, int numberOfCoefficients, int numberOfConstrainedDerivatives,
                                                    Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2,
                                                    List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList, double icpPositionFinalSegment2Scalar)
   {
      calculateGeneralizedICPMatricesOnCMPSegment2(omega0, 0, cmpPolynomialSegment2);

      // TODO: check whether division always integer
      for (int i = 0; i < numberOfConstrainedDerivatives; i++)
      {
         double icpQuantityInitialConditionScalar = icpQuantityInitialConditionList.get(i).getElement(direction.getIndex());
         calculateGeneralizedICPMatricesOnCMPSegment1(omega0, i, cmpPolynomialSegment1);
         setGeneralizedBoundaryConstraints(i, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2,
                                           icpQuantityInitialConditionScalar, icpPositionFinalSegment2Scalar);
      }
   }

   private double generalizedBoundaryConditionValue = Double.NaN;
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix1Transpose = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix2Transpose = new DenseMatrix64F(defaultSize, 1);

   private void setGeneralizedBoundaryConstraintICP0(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives,
                                                     double icpQuantityInitialConditionScalar, double icpPositionFinalSegment2,
                                                     DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1, DenseMatrix64F generalizedGammaPrimeMatrixSegment1,
                                                     DenseMatrix64F alphaBetaPrimeRowSegment2, DenseMatrix64F gammaPrimeMatrixSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();

      generalizedBoundaryConditionValue = icpQuantityInitialConditionScalar
            - generalizedGammaPrimeMatrixSegment1.get(0, 0) * gammaPrimeMatrixSegment2.get(0, 0) * icpPositionFinalSegment2;
      boundaryConditionVector.set(order, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1.set(generalizedAlphaBetaPrimeRowSegment1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order, 0);

      CommonOps.scale(generalizedGammaPrimeMatrixSegment1.get(0, 0), alphaBetaPrimeRowSegment2, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order, numberOfCoefficients);
   }

   private void setGeneralizedBoundaryConstraintCMP0(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1)
   {
      resetGeneralizedBoundaryConditionContainers();

      double tInitial1 = cmpPolynomialSegment1.getInitialTime();

      generalizedBoundaryConditionValue = cmpPolynomialSegment1.getDerivative(order, tInitial1);
      boundaryConditionVector.set(order + numberOfConstrainedDerivatives, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1Transpose.set(cmpPolynomialSegment1.getXPowersDerivativeVector(order, tInitial1));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix1Transpose, generalizedBoundaryConditionSubMatrix1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order + numberOfConstrainedDerivatives, 0);

      generalizedBoundaryConditionSubMatrix2Transpose.zero();
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix2Transpose, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order + numberOfConstrainedDerivatives, numberOfCoefficients);
   }

   private void setGeneralizedBoundaryConstraintCMP1(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1,
                                                     Trajectory cmpPolynomialSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();

      double tFinal1 = cmpPolynomialSegment1.getFinalTime();
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();

      generalizedBoundaryConditionValue = 0.0;
      boundaryConditionVector.set(order + 2 * numberOfConstrainedDerivatives, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1Transpose.set(cmpPolynomialSegment1.getXPowersDerivativeVector(order, tFinal1));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix1Transpose, generalizedBoundaryConditionSubMatrix1);
      CommonOps.scale(-1.0, generalizedBoundaryConditionSubMatrix1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order + 2 * numberOfConstrainedDerivatives, 0);

      generalizedBoundaryConditionSubMatrix2Transpose.set(cmpPolynomialSegment2.getXPowersDerivativeVector(order, tInitial2));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix2Transpose, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order + 2 * numberOfConstrainedDerivatives, numberOfCoefficients);
   }

   private void setGeneralizedBoundaryConstraintCMP2(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();

      double tFinal2 = cmpPolynomialSegment2.getFinalTime();

      generalizedBoundaryConditionValue = cmpPolynomialSegment2.getDerivative(order, tFinal2);
      boundaryConditionVector.set(order + 3 * numberOfConstrainedDerivatives, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1Transpose.zero();
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix1Transpose, generalizedBoundaryConditionSubMatrix1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order + 3 * numberOfConstrainedDerivatives, 0);

      generalizedBoundaryConditionSubMatrix2Transpose.set(cmpPolynomialSegment2.getXPowersDerivativeVector(order, tFinal2));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix2Transpose, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order + 3 * numberOfConstrainedDerivatives, numberOfCoefficients);
   }

   private void computeAdjustedPolynomialCoefficientVectors1D(int numberOfCoefficients)
   {
      // Uses the Moore-Penrose pseudo-inverse to counter bad conditioning of boundaryConditionMatrix
      pseudoInverseSolver.setA(boundaryConditionMatrix);
      pseudoInverseSolver.solve(boundaryConditionVector, polynomialCoefficientCombinedVectorAdjustment);

      polynomialCoefficientVectorAdjustmentSegment1.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, 0, numberOfCoefficients, 0, 1));
      polynomialCoefficientVectorAdjustmentSegment2.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, numberOfCoefficients,
                                                                          2 * numberOfCoefficients, 0, 1));
   }

   private void calculateGeneralizedICPMatricesOnCMPSegment2(double omega0, int derivativeOrder, Trajectory cmpPolynomialSegment2)
   {
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();
      icpToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial2, 0, cmpPolynomialSegment2, alphaPrimeRowSegment2, betaPrimeRowSegment2,
                                                                 gammaPrimeMatrixSegment2, alphaBetaPrimeRowSegment2);
   }

   private void calculateGeneralizedICPMatricesOnCMPSegment1(double omega0, int derivativeOrder, Trajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();
      icpToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial1, derivativeOrder, cmpPolynomialSegment1, generalizedAlphaPrimeRowSegment1,
                                                                 generalizedBetaPrimeRowSegment1, generalizedGammaPrimeMatrixSegment1,
                                                                 generalizedAlphaBetaPrimeRowSegment1);
   }

   private void setGeneralizedBoundaryConstraints(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1,
                                                  Trajectory cmpPolynomialSegment2, double icpQuantityInitialConditionScalar,
                                                  double icpPositionFinalSegment2Scalar)
   {
      setGeneralizedBoundaryConstraintICP0(order, numberOfCoefficients, numberOfConstrainedDerivatives, icpQuantityInitialConditionScalar,
                                           icpPositionFinalSegment2Scalar, generalizedAlphaBetaPrimeRowSegment1, generalizedGammaPrimeMatrixSegment1,
                                           alphaBetaPrimeRowSegment2, gammaPrimeMatrixSegment2);
      setGeneralizedBoundaryConstraintCMP0(order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1);
      setGeneralizedBoundaryConstraintCMP1(order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2);
      setGeneralizedBoundaryConstraintCMP2(order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment2);
   }

   private void initializeMatrices1D(int numberOfCoefficients, int numberOfConstrainedDerivatives)
   {
      boundaryConditionMatrix.reshape(4 * numberOfConstrainedDerivatives, 2 * numberOfCoefficients);
      boundaryConditionMatrixInverse.reshape(2 * numberOfCoefficients, 4 * numberOfConstrainedDerivatives);
      boundaryConditionVector.reshape(4 * numberOfConstrainedDerivatives, 1);
      polynomialCoefficientCombinedVectorAdjustment.reshape(2 * numberOfCoefficients, 1);

      polynomialCoefficientVectorAdjustmentSegment1.reshape(numberOfCoefficients, 1);
      polynomialCoefficientVectorAdjustmentSegment2.reshape(numberOfCoefficients, 1);

      generalizedBoundaryConditionSubMatrix1.reshape(1, numberOfCoefficients);
      generalizedBoundaryConditionSubMatrix1Transpose.reshape(numberOfCoefficients, 1);
      generalizedBoundaryConditionSubMatrix2.reshape(1, numberOfCoefficients);
      generalizedBoundaryConditionSubMatrix2Transpose.reshape(numberOfCoefficients, 1);

      generalizedAlphaPrimeRowSegment1.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRowSegment1.reshape(1, numberOfCoefficients);
      generalizedGammaPrimeMatrixSegment1.reshape(1, 1);
      generalizedAlphaBetaPrimeRowSegment1.reshape(1, numberOfCoefficients);

      alphaPrimeRowSegment2.reshape(1, numberOfCoefficients);
      betaPrimeRowSegment2.reshape(1, numberOfCoefficients);
      gammaPrimeMatrixSegment2.reshape(1, 1);
      alphaBetaPrimeRowSegment2.reshape(1, numberOfCoefficients);
   }

   private void resetGeneralizedBoundaryConditionContainers()
   {
      generalizedBoundaryConditionValue = Double.NaN;
      generalizedBoundaryConditionSubMatrix1.zero();
      generalizedBoundaryConditionSubMatrix2.zero();
      generalizedBoundaryConditionSubMatrix1Transpose.zero();
      generalizedBoundaryConditionSubMatrix2Transpose.zero();
   }
}
