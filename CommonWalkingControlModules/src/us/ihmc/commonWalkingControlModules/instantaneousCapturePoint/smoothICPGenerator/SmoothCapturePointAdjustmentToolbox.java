package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;

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
   
   private final FramePoint cmp10 = new FramePoint();
   private final FramePoint cmp11 = new FramePoint();
   private final FramePoint cmp21 = new FramePoint();
   private final FramePoint cmp22 = new FramePoint();
   
   private List<FrameTuple<?, ?>> icpQuantityInitialSegment1List = new ArrayList<FrameTuple<?, ?>>();
   private FrameTuple<?, ?> icpQuantityInitialSegment1 = new FramePoint();
   
   public void adjustDesiredTrajectoriesForInitialSmoothing(List<FramePoint> entryCornerPointsToPack, List<FramePoint> exitCornerPointsToPack,
                                                                   List<YoFrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(0);
      double tInitial = cmpPolynomial3D.getInitialTime();
      
      // TODO: check whether better ways of setting it exist
      for(int i = 0; i < cmpPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
      {
         cmpPolynomial3D.getDerivative(i, tInitial, icpQuantityInitialSegment1);
         icpQuantityInitialSegment1List.add(icpQuantityInitialSegment1);
      }
      
      initializeMatrices1D(cmpPolynomials3D.get(0).getNumberOfCoefficients());
      adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3D, icpQuantityInitialSegment1List, entryCornerPointsToPack, exitCornerPointsToPack);
   }
   
   public void adjustDesiredTrajectoriesForInitialSmoothing3D(double omega0, List<YoFrameTrajectory3D> cmpPolynomials3D, List<FrameTuple<?, ?>> icpQuantityInitialSegment1List,
                                                                     List<FramePoint> entryCornerPointsToPack, List<FramePoint> exitCornerPointsToPack)
   {
      YoFrameTrajectory3D cmpPolynomial3DSegment1 = cmpPolynomials3D.get(0);
      YoFrameTrajectory3D cmpPolynomial3DSegment2 = cmpPolynomials3D.get(1);
      
//      // this is very hack-ish
//      cmpPolynomial3DSegment1.getFramePositionInitial(cmp10);
//      cmpPolynomial3DSegment1.getFramePositionFinal(cmp11);
//      cmpPolynomial3DSegment2.getFramePositionInitial(cmp21);
//      cmpPolynomial3DSegment2.getFramePositionFinal(cmp22);
//      
//      double tAdjusted1 = 0.9 * (cmpPolynomial3DSegment2.getFinalTime() - cmpPolynomial3DSegment1.getInitialTime());
//      cmpPolynomial3DSegment1.setLinear(cmpPolynomial3DSegment1.getInitialTime(), tAdjusted1, cmp10, cmp11);
//      cmpPolynomial3DSegment2.setLinear(tAdjusted1, cmpPolynomial3DSegment2.getFinalTime(), cmp21, cmp22);

      FramePoint icpPositionFinalSegment2 = exitCornerPointsToPack.get(1);

      for(Direction direction : Direction.values())
      {
         YoTrajectory cmpPolynomialSegment1 = cmpPolynomial3DSegment1.getYoTrajectory(direction);
         YoTrajectory cmpPolynomialSegment2 = cmpPolynomial3DSegment2.getYoTrajectory(direction);
         
         double icpPositionFinalSegment2Scalar = icpPositionFinalSegment2.get(direction);
         
         populateBoundaryConditionMatrices1D(omega0, direction, cmpPolynomialSegment1, cmpPolynomialSegment2, icpQuantityInitialSegment1List, icpPositionFinalSegment2Scalar);
         computeAdjustedPolynomialCoefficientVectors1D();
         adjustCMPPolynomials(cmpPolynomialSegment1, cmpPolynomialSegment2);
      }
      SmoothCapturePointTools.computeDesiredCornerPoints(entryCornerPointsToPack, exitCornerPointsToPack, cmpPolynomials3D, omega0);
      
      double t0 = cmpPolynomial3DSegment1.getInitialTime();
      double t1 = cmpPolynomial3DSegment1.getFinalTime();
      double t2 = cmpPolynomial3DSegment2.getFinalTime();
      
      PrintTools.debug("Timings = (" + t0 + ", " + t1 + ", " + t2 + ")");
      cmpPolynomial3DSegment1.compute(t0);
      PrintTools.debug("CMP10 = " + cmpPolynomial3DSegment1.getPosition().toString());
      cmpPolynomial3DSegment1.compute(t1);
      PrintTools.debug("CMP11 = " + cmpPolynomial3DSegment1.getPosition().toString());
      cmpPolynomial3DSegment2.compute(t1);
      PrintTools.debug("CMP21 = " + cmpPolynomial3DSegment2.getPosition().toString());
      cmpPolynomial3DSegment2.compute(t2);
      PrintTools.debug("CMP22 = " + cmpPolynomial3DSegment2.getPosition().toString());
   }   
   
   private void adjustCMPPolynomials(YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2)
   {
      cmpPolynomialSegment1.setDirectly(polynomialCoefficientVectorAdjustmentSegment1);
      cmpPolynomialSegment2.setDirectly(polynomialCoefficientVectorAdjustmentSegment2);
   }
   
   private void populateBoundaryConditionMatrices1D(double omega0, Direction direction, YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2, 
                                                          List<FrameTuple<?, ?>> icpQuantityInitialSegment1List, double icpPositionFinalSegment2Scalar)
   {    
      int numberOfCoefficients = cmpPolynomialSegment1.getNumberOfCoefficients();
      int numberOfConstrainedDerivatives = numberOfCoefficients / 2;
      
      calculateGeneralizedICPMatricesOnCMPSegment2(omega0, 0, cmpPolynomialSegment2);
      
      // TODO: check whether division always integer
      for(int i = 0; i < numberOfConstrainedDerivatives; i++)
      {
         double icpQuantityInitialSegment1Scalar = icpQuantityInitialSegment1List.get(i).get(direction);
         calculateGeneralizedICPMatricesOnCMPSegment1(omega0, i, cmpPolynomialSegment1);
         setGeneralizedBoundaryConstraints(i, numberOfCoefficients, cmpPolynomialSegment1, cmpPolynomialSegment2,
                                           icpQuantityInitialSegment1Scalar, icpPositionFinalSegment2Scalar);
      }
   }
   
   private  double generalizedBoundaryConditionValue = Double.NaN;
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix1Transpose = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F generalizedBoundaryConditionSubMatrix2Transpose = new DenseMatrix64F(defaultSize, 1);
   
   private void setGeneralizedBoundaryConstraintICP0(int order, int numberOfCoefficients, double icpQuantityInitialSegment1, double icpPositionFinalSegment2,
                                                           DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1, DenseMatrix64F generalizedGammaPrimeMatrixSegment1,
                                                           DenseMatrix64F alphaBetaPrimeRowSegment2, DenseMatrix64F gammaPrimeMatrixSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();
      
      generalizedBoundaryConditionValue = icpQuantityInitialSegment1 - generalizedGammaPrimeMatrixSegment1.get(0, 0) * gammaPrimeMatrixSegment2.get(0, 0) * icpPositionFinalSegment2;
      boundaryConditionVector.set(order, 0, generalizedBoundaryConditionValue);
      
      generalizedBoundaryConditionSubMatrix1.set(generalizedAlphaBetaPrimeRowSegment1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order, 0);
      
      CommonOps.scale(generalizedGammaPrimeMatrixSegment1.get(0, 0), alphaBetaPrimeRowSegment2, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order, numberOfCoefficients);
   }
   
   private void setGeneralizedBoundaryConstraintCMP0(int order, int numberOfCoefficients, YoTrajectory cmpPolynomialSegment1)
   {
      resetGeneralizedBoundaryConditionContainers();
      
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();
      
      generalizedBoundaryConditionValue = cmpPolynomialSegment1.getDerivative(order, tInitial1);
      boundaryConditionVector.set(order + numberOfCoefficients/2, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1Transpose.set(cmpPolynomialSegment1.getXPowersDerivativeVector(order, tInitial1));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix1Transpose, generalizedBoundaryConditionSubMatrix1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order + numberOfCoefficients/2, 0);
      
      generalizedBoundaryConditionSubMatrix2Transpose.zero();
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix2Transpose, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order + numberOfCoefficients/2, numberOfCoefficients);
   }
   
   private void setGeneralizedBoundaryConstraintCMP1(int order, int numberOfCoefficients, YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();
      
      double tFinal1 = cmpPolynomialSegment1.getFinalTime();
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();
      
      generalizedBoundaryConditionValue = 0.0;
      boundaryConditionVector.set(order + numberOfCoefficients, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1Transpose.set(cmpPolynomialSegment1.getXPowersDerivativeVector(order, tFinal1));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix1Transpose, generalizedBoundaryConditionSubMatrix1);
      CommonOps.scale(-1.0, generalizedBoundaryConditionSubMatrix1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order + numberOfCoefficients, 0);
      
      generalizedBoundaryConditionSubMatrix2Transpose.set(cmpPolynomialSegment2.getXPowersDerivativeVector(order, tInitial2));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix2Transpose, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order + numberOfCoefficients, numberOfCoefficients);
   }
   
   private void setGeneralizedBoundaryConstraintCMP2(int order, int numberOfCoefficients, YoTrajectory cmpPolynomialSegment2)
   {
      resetGeneralizedBoundaryConditionContainers();
      
      double tFinal2 = cmpPolynomialSegment2.getFinalTime();
      
      generalizedBoundaryConditionValue = cmpPolynomialSegment2.getDerivative(order, tFinal2);
      boundaryConditionVector.set(order + numberOfCoefficients + numberOfCoefficients/2, 0, generalizedBoundaryConditionValue);

      generalizedBoundaryConditionSubMatrix1Transpose.zero();
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix1Transpose, generalizedBoundaryConditionSubMatrix1);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix1, boundaryConditionMatrix, order + numberOfCoefficients + numberOfCoefficients/2, 0);
      
      generalizedBoundaryConditionSubMatrix2Transpose.set(cmpPolynomialSegment2.getXPowersDerivativeVector(order, tFinal2));
      CommonOps.transpose(generalizedBoundaryConditionSubMatrix2Transpose, generalizedBoundaryConditionSubMatrix2);
      CommonOps.insert(generalizedBoundaryConditionSubMatrix2, boundaryConditionMatrix, order + numberOfCoefficients + numberOfCoefficients/2, numberOfCoefficients);
   }
   
   private void computeAdjustedPolynomialCoefficientVectors1D()
   {
      // Uses the Moore-Penrose pseudo-inverse to counter bad conditioning of boundaryConditionMatrix
      CommonOps.pinv(boundaryConditionMatrix, boundaryConditionMatrixInverse);
      CommonOps.mult(boundaryConditionMatrixInverse, boundaryConditionVector, polynomialCoefficientCombinedVectorAdjustment);
      
      int numberOfCoefficients = polynomialCoefficientCombinedVectorAdjustment.numRows / 2; 
      polynomialCoefficientVectorAdjustmentSegment1.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, 0, numberOfCoefficients, 0, 1));
      polynomialCoefficientVectorAdjustmentSegment2.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, numberOfCoefficients, 2*numberOfCoefficients, 0, 1));
      
      PrintTools.debug("BC Matrix = " + boundaryConditionMatrix.toString());
      PrintTools.debug("BC MInver = " + boundaryConditionMatrixInverse.toString());
      PrintTools.debug("BC Vector = " + boundaryConditionVector.toString());
      PrintTools.debug("PC adjust = " + polynomialCoefficientCombinedVectorAdjustment.toString());
      PrintTools.debug("");
   }
   
   private void calculateGeneralizedICPMatricesOnCMPSegment2(double omega0, int derivativeOrder, YoTrajectory cmpPolynomialSegment2)
   {
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();
      SmoothCapturePointTools.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial2, 0, cmpPolynomialSegment2, alphaPrimeRowSegment2, betaPrimeRowSegment2, 
                                                                              gammaPrimeMatrixSegment2, alphaBetaPrimeRowSegment2);
   }
   
   private void calculateGeneralizedICPMatricesOnCMPSegment1(double omega0, int derivativeOrder, YoTrajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();
      SmoothCapturePointTools.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial1, derivativeOrder, cmpPolynomialSegment1, generalizedAlphaPrimeRowSegment1, 
                                                                              generalizedBetaPrimeRowSegment1, generalizedGammaPrimeMatrixSegment1, generalizedAlphaBetaPrimeRowSegment1);
   }
   
   private void setGeneralizedBoundaryConstraints(int order, int numberOfCoefficients, YoTrajectory cmpPolynomialSegment1, YoTrajectory cmpPolynomialSegment2,
                                                        double icpQuantityInitialSegment1Scalar, double icpPositionFinalSegment2Scalar)
   {
      setGeneralizedBoundaryConstraintICP0(order, numberOfCoefficients, icpQuantityInitialSegment1Scalar, icpPositionFinalSegment2Scalar,
                                           generalizedAlphaBetaPrimeRowSegment1, generalizedGammaPrimeMatrixSegment1,
                                           alphaBetaPrimeRowSegment2, gammaPrimeMatrixSegment2);
      setGeneralizedBoundaryConstraintCMP0(order, numberOfCoefficients, cmpPolynomialSegment1);
      setGeneralizedBoundaryConstraintCMP1(order, numberOfCoefficients, cmpPolynomialSegment1, cmpPolynomialSegment2);
      setGeneralizedBoundaryConstraintCMP2(order, numberOfCoefficients, cmpPolynomialSegment2);
   }
   
   private void initializeMatrices1D(int numberOfCoefficients)
   {  
      boundaryConditionMatrix.reshape(2 * numberOfCoefficients, 2 * numberOfCoefficients);
      boundaryConditionMatrixInverse.reshape(2 * numberOfCoefficients, 2 * numberOfCoefficients);
      boundaryConditionVector.reshape(2 * numberOfCoefficients, 1);
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
