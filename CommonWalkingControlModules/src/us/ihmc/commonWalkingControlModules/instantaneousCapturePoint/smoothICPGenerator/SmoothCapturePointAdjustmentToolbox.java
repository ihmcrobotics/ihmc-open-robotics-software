package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoTrajectory;

public class SmoothCapturePointAdjustmentToolbox
{
   private static final int defaultSize = 1000;
   
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
   
   private List<FrameTuple<?, ?>> icpQuantityInitialConditionList = new ArrayList<FrameTuple<?, ?>>();
   private FrameTuple<?, ?> icpQuantityInitialSegment1 = new FramePoint();
   
   private final SmoothCapturePointToolbox icpToolbox;
   
   public SmoothCapturePointAdjustmentToolbox(SmoothCapturePointToolbox smoothCapturePointToolbox)
   {
      this.icpToolbox = smoothCapturePointToolbox;
      
      icpQuantityInitialConditionList.add(new FramePoint());
      while(icpQuantityInitialConditionList.size() < defaultSize)
      {
         icpQuantityInitialConditionList.add(new FrameVector());
      }
   }
   
   public void setICPInitialConditions(List<FramePoint> exitCornerPointsToPack, List<YoFrameTrajectory3D> cmpPolynomials3D, 
                                        int numberOfSegmentsSwing0, boolean isInitialTransfer, double omega0)
   {
      if(isInitialTransfer)
      {
         YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(0);
         for(int i = 0; i < cmpPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);
            
            cmpPolynomial3D.getDerivative(i, cmpPolynomial3D.getInitialTime(), icpQuantityInitialCondition);
         }
      }
      else
      {
         YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(numberOfSegmentsSwing0 - 1);
         for(int i = 0; i < cmpPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);
            
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, cmpPolynomial3D.getFinalTime(), i, cmpPolynomial3D, 
                                                                            exitCornerPointsToPack.get(numberOfSegmentsSwing0 - 1), 
                                                                            icpQuantityInitialCondition);
         }         
      }
   }
   
   public void adjustDesiredTrajectoriesForInitialSmoothing(List<FramePoint> entryCornerPointsToPack, List<FramePoint> exitCornerPointsToPack,
                                                            List<YoFrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3D, icpQuantityInitialConditionList, entryCornerPointsToPack, exitCornerPointsToPack);
   }
   
   public void adjustDesiredTrajectoriesForInitialSmoothing3D(double omega0, List<YoFrameTrajectory3D> cmpPolynomials3D, List<FrameTuple<?, ?>> icpQuantityInitialConditionList,
                                                                     List<FramePoint> entryCornerPointsToPack, List<FramePoint> exitCornerPointsToPack)
   {
      YoFrameTrajectory3D cmpPolynomial3DSegment1 = cmpPolynomials3D.get(0);
      YoFrameTrajectory3D cmpPolynomial3DSegment2 = cmpPolynomials3D.get(1);

      FramePoint icpPositionFinalSegment2 = exitCornerPointsToPack.get(1);

      for(Direction direction : Direction.values())
      {
         YoTrajectory cmpPolynomialSegment1 = cmpPolynomial3DSegment1.getYoTrajectory(direction);
         YoTrajectory cmpPolynomialSegment2 = cmpPolynomial3DSegment2.getYoTrajectory(direction);
         
         double icpPositionFinalSegment2Scalar = icpPositionFinalSegment2.get(direction);
                 
         int numberOfCoefficients = cmpPolynomialSegment1.getNumberOfCoefficients();
         int numberOfConstrainedDerivatives = cmpPolynomialSegment1.getNumberOfCoefficients() / 2;

         initializeMatrices1D(numberOfCoefficients, numberOfConstrainedDerivatives);
         
         populateBoundaryConditionMatrices1D(omega0, direction, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2, icpQuantityInitialConditionList, icpPositionFinalSegment2Scalar);
         computeAdjustedPolynomialCoefficientVectors1D(numberOfCoefficients);
         adjustCMPPolynomials(cmpPolynomialSegment1, cmpPolynomialSegment2);
      }
      icpToolbox.computeDesiredCornerPoints(entryCornerPointsToPack, exitCornerPointsToPack, cmpPolynomials3D, omega0);
   }   
   
   private void adjustCMPPolynomials(YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2)
   {
      cmpPolynomialSegment1.setDirectly(polynomialCoefficientVectorAdjustmentSegment1);
      cmpPolynomialSegment2.setDirectly(polynomialCoefficientVectorAdjustmentSegment2);
   }

   private void populateBoundaryConditionMatrices1D(double omega0, Direction direction, int numberOfCoefficients, int numberOfConstrainedDerivatives, YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2, 
                                                          List<FrameTuple<?, ?>> icpQuantityInitialConditionList, double icpPositionFinalSegment2Scalar)
   {    
      calculateGeneralizedICPMatricesOnCMPSegment2(omega0, 0, cmpPolynomialSegment2);
      
      // TODO: check whether division always integer
      for(int i = 0; i < numberOfConstrainedDerivatives; i++)
      {
         double icpQuantityInitialConditionScalar = icpQuantityInitialConditionList.get(i).get(direction);
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
   
   private void setGeneralizedBoundaryConstraintICP0(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, double icpQuantityInitialConditionScalar, double icpPositionFinalSegment2,
                                                           DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1, DenseMatrix64F generalizedGammaPrimeMatrixSegment1,
                                                           DenseMatrix64F alphaBetaPrimeRowSegment2, DenseMatrix64F gammaPrimeMatrixSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();
      
      generalizedBoundaryConditionValue = icpQuantityInitialConditionScalar - generalizedGammaPrimeMatrixSegment1.get(0, 0) * gammaPrimeMatrixSegment2.get(0, 0) * icpPositionFinalSegment2;
      boundaryConditionVector.set(order, 0, generalizedBoundaryConditionValue);
      
      generalizedBoundaryConditionSubMatrix1.set(generalizedAlphaBetaPrimeRowSegment1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order, 0);
      
      CommonOps.scale(generalizedGammaPrimeMatrixSegment1.get(0, 0), alphaBetaPrimeRowSegment2, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order, numberOfCoefficients);
   }
   
   private void setGeneralizedBoundaryConstraintCMP0(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, YoTrajectory cmpPolynomialSegment1)
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
   
   private void setGeneralizedBoundaryConstraintCMP1(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives,  YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2)
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
   
   private void setGeneralizedBoundaryConstraintCMP2(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, YoTrajectory cmpPolynomialSegment2)
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
      polynomialCoefficientVectorAdjustmentSegment2.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, numberOfCoefficients, 2*numberOfCoefficients, 0, 1));
   }
   
   private void calculateGeneralizedICPMatricesOnCMPSegment2(double omega0, int derivativeOrder, YoTrajectory cmpPolynomialSegment2)
   {
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();
      icpToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial2, 0, cmpPolynomialSegment2, alphaPrimeRowSegment2, betaPrimeRowSegment2, 
                                                                              gammaPrimeMatrixSegment2, alphaBetaPrimeRowSegment2);
   }
   
   private void calculateGeneralizedICPMatricesOnCMPSegment1(double omega0, int derivativeOrder, YoTrajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();
      icpToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial1, derivativeOrder, cmpPolynomialSegment1, generalizedAlphaPrimeRowSegment1, 
                                                                              generalizedBetaPrimeRowSegment1, generalizedGammaPrimeMatrixSegment1, generalizedAlphaBetaPrimeRowSegment1);
   }
   
   private void setGeneralizedBoundaryConstraints(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2,
                                                  double icpQuantityInitialConditionScalar, double icpPositionFinalSegment2Scalar)
   {
      setGeneralizedBoundaryConstraintICP0(order, numberOfCoefficients, numberOfConstrainedDerivatives,  icpQuantityInitialConditionScalar, icpPositionFinalSegment2Scalar,
                                           generalizedAlphaBetaPrimeRowSegment1, generalizedGammaPrimeMatrixSegment1,
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
