package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;

public class ICPPlannerTrajectoryFromCMPPolynomialGenerator implements PositionTrajectoryGenerator
{
   private final DoubleYoVariable omega0;
   
   private final List<YoPolynomial3D> cmpPolynomialTrajectories = new ArrayList<>();
   
   private final List<FramePoint> desiredICPBoundaryPositions = new ArrayList<>();
   
   private final FramePoint desiredICPPositionOutput = new FramePoint();
   private final FrameVector desiredICPVelocityOutput = new FrameVector();
   
   private final DenseMatrix64F coefficientsCombinedVector = new DenseMatrix64F();
   private final DenseMatrix64F coefficientsCurrentVector = new DenseMatrix64F();
   
   Point3D icpDesiredCurrent = new Point3D();
   Point3D icpDesiredFinal = new Point3D();
   
   private DenseMatrix64F identityMatrix = new DenseMatrix64F();
   private DenseMatrix64F gammaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F backwardIterationMatrix = new DenseMatrix64F();
   private DenseMatrix64F alphaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F betaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F alphaBetaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F finalConstraintMatrix = new DenseMatrix64F();
   private DenseMatrix64F tPowersMatrix = new DenseMatrix64F();
   private DenseMatrix64F polynomialCoefficientVector = new DenseMatrix64F();
   
   private DenseMatrix64F icpDesiredInitialVector = new DenseMatrix64F(10000, 1);
   private DenseMatrix64F icpDesiredFinalVector = new DenseMatrix64F(10000, 1);
   
   // Pre-allocated helper matrices
   private final DenseMatrix64F M1 = new DenseMatrix64F(10000, 10000);
   private final DenseMatrix64F M2 = new DenseMatrix64F(10000, 10000);
   private final DenseMatrix64F M3 = new DenseMatrix64F(10000, 10000);
   
   private final int numberOfSegments;
      
   public ICPPlannerTrajectoryFromCMPPolynomialGenerator(DoubleYoVariable omega0, List<YoPolynomial3D> cmpPolynomialTrajectories)
   {
      this.omega0 = omega0;
      numberOfSegments = cmpPolynomialTrajectories.size();
      
      icpDesiredInitialVector.reshape(numberOfSegments, 3);
      icpDesiredFinalVector.reshape(numberOfSegments, 3);
      
      
      
      for(int i = 0; i < cmpPolynomialTrajectories.size(); ++i)
      {
         this.cmpPolynomialTrajectories.add(cmpPolynomialTrajectories.get(i));
      }
   }
   
   public void initialize()
   {
      calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar();
   }

   public void compute(double time)
   {
      initialize();
      calculateICPDesiredCurrentFromCMPPolynomialsScalar(time);
   }
   
   // desiredICPCurrent: desired ICP at time; modified.
   // TODO: take care of frame matching
   private void calculateICPFromCorrespondingCMPPolynomial(YoFramePoint desiredICPCurrent, YoFramePoint desiredICPFinal, YoPolynomial3D cmpPolynomialTrajectory, double time, double trajectoryTime)
   {
      Point3D desiredICPCurrentPosition = new Point3D();
      desiredICPCurrent.getPoint(desiredICPCurrentPosition);
      
      Point3D desiredICPFinalPosition = new Point3D();
      desiredICPFinal.getPoint(desiredICPFinalPosition);
      
      calculateICPFromCorrespondingCMPPolynomial(desiredICPCurrentPosition, desiredICPFinalPosition, cmpPolynomialTrajectory, time, trajectoryTime);
      
      desiredICPCurrent.setPoint(desiredICPCurrentPosition);
   }
   
   private void calculateICPFromCorrespondingCMPPolynomial(Point3D desiredICPCurrent, Point3D desiredICPFinal, YoPolynomial3D cmpPolynomialTrajectory, double time, double trajectoryTime)
   {     
      double desiredICPX = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getX(), cmpPolynomialTrajectory.getYoPolynomialX(), time, trajectoryTime);
      double desiredICPY = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getY(), cmpPolynomialTrajectory.getYoPolynomialY(), time, trajectoryTime);
      double desiredICPZ = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getZ(), cmpPolynomialTrajectory.getYoPolynomialZ(), time, trajectoryTime);
      
      desiredICPCurrent.set(new Point3D(desiredICPX, desiredICPY, desiredICPZ));
   }
   
   private double calculateICPFromCorrespondingCMPPolynomial1D(double desiredICPFinalPosition1D, YoPolynomial cmpPolynomialTrajectory1D, double time, double trajectoryTime)
   {      
      double exponentialFactor = Math.exp(omega0.getDoubleValue()*(time-trajectoryTime));
      
      double desiredICPCurrentPosition1D = desiredICPFinalPosition1D * exponentialFactor;  
      for(int i = 0; i < cmpPolynomialTrajectory1D.getNumberOfCoefficients(); ++i)
      {
         desiredICPCurrentPosition1D += 1/Math.pow(omega0.getDoubleValue(), i) * (cmpPolynomialTrajectory1D.getDerivative(i, time) - cmpPolynomialTrajectory1D.getDerivative(i, trajectoryTime) * exponentialFactor);
      }
      
      return desiredICPCurrentPosition1D;
   }
   
   
   // SCALAR FORMULATION
   private void calculateICPDesiredCurrentFromCMPPolynomialsScalar(double time)
   {
      int initialSegment = 0;
      
      icpDesiredFinal.set(icpDesiredFinalVector.get(initialSegment, 0), icpDesiredFinalVector.get(initialSegment, 1), icpDesiredFinalVector.get(initialSegment, 2));
      
      icpDesiredCurrent.setX(calculateICPFromCorrespondingCMPPolynomialScalarX(icpDesiredFinal.getX(), initialSegment, time));
      icpDesiredCurrent.setY(calculateICPFromCorrespondingCMPPolynomialScalarY(icpDesiredFinal.getY(), initialSegment, time)); 
      icpDesiredCurrent.setZ(calculateICPFromCorrespondingCMPPolynomialScalarZ(icpDesiredFinal.getZ(), initialSegment, time)); 
   }
   
   private void calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar()
   {       
      setICPTerminalConditionScalar();
      
      for(int i = numberOfSegments-1; i >= 0; i--)
      {
         icpDesiredInitialVector.set(i, 0, calculateICPFromCorrespondingCMPPolynomialScalarX(icpDesiredFinalVector.get(i, 0), i, 0.0));
         icpDesiredInitialVector.set(i, 1, calculateICPFromCorrespondingCMPPolynomialScalarY(icpDesiredFinalVector.get(i, 1), i, 0.0));
         icpDesiredInitialVector.set(i, 2, calculateICPFromCorrespondingCMPPolynomialScalarZ(icpDesiredFinalVector.get(i, 2), i, 0.0));
         
         if(i > 0)
         {
            icpDesiredFinalVector.set(i-1, 0, icpDesiredInitialVector.get(i, 0));
            icpDesiredFinalVector.set(i-1, 1, icpDesiredInitialVector.get(i, 1));
            icpDesiredFinalVector.set(i-1, 2, icpDesiredInitialVector.get(i, 2));
         }
      }
   }
   
   private void setICPTerminalConditionScalar()
   {
      for(int i = 0; i < 3; i++)
      {
         YoPolynomial cmPolynomialFinalSegment = cmpPolynomialTrajectories.get(numberOfSegments-1).getYoPolynomial(i);
         cmPolynomialFinalSegment.compute(cmPolynomialFinalSegment.getXFinal());
         icpDesiredFinalVector.set(numberOfSegments-1, i, cmPolynomialFinalSegment.getPosition());
      }
   }
   
   
   
   private double calculateICPFromCorrespondingCMPPolynomialScalar(YoPolynomial cmpPolynomial, double icpDesiredFinal, double time)
   {
      double icpDesired = 0.0;
         
      double timeTrajectory = cmpPolynomial.getXFinal();
      DenseMatrix64F polynomialCoefficientVector =  cmpPolynomial.getCoefficientsVector();
         
      calculateAlphaPrimeOnCMPSegment(alphaPrimeMatrix, cmpPolynomial, time);
      calculateBetaPrimeOnCMPSegment(betaPrimeMatrix, cmpPolynomial, time, timeTrajectory);
      calculateGammaPrimeOnCMPSegment(gammaPrimeMatrix, time, timeTrajectory);
      CommonOps.subtract(alphaPrimeMatrix, betaPrimeMatrix, alphaBetaPrimeMatrix);
      
      M1.reshape(alphaBetaPrimeMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
      M1.zero();
      M2.reshape(gammaPrimeMatrix.getNumRows(), gammaPrimeMatrix.getNumCols());
      M2.zero();
      
      CommonOps.mult(alphaBetaPrimeMatrix, polynomialCoefficientVector, M1);
      CommonOps.scale(icpDesiredFinal, gammaPrimeMatrix, M2);
      
      CommonOps.addEquals(M1, M2);
      
      icpDesired = M1.get(0, 0);
      return icpDesired;
   }
   
   private double calculateICPFromCorrespondingCMPPolynomialScalarX(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();

      return calculateICPFromCorrespondingCMPPolynomialScalar(cmpPolynomialX, icpDesiredFinal, time);
   }
   
   private double calculateICPFromCorrespondingCMPPolynomialScalarY(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();

      return calculateICPFromCorrespondingCMPPolynomialScalar(cmpPolynomialY, icpDesiredFinal, time);
   }
   
   private double calculateICPFromCorrespondingCMPPolynomialScalarZ(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();

      return calculateICPFromCorrespondingCMPPolynomialScalar(cmpPolynomialZ, icpDesiredFinal, time);
   }
   
   
   // MATRIX FORMULATION
   // ICP = (M1)^(-1) * M2 * P = M3 * P
   private void calculateICPFromCorrespondingCMPPolynomialMatrix(DenseMatrix64F icpVector, DenseMatrix64F identityMatrix, DenseMatrix64F gammaPrimeMatrix, DenseMatrix64F backwardIterationMatrix,
                                                                DenseMatrix64F alphaBetaPrimeMatrix, DenseMatrix64F terminalConstraintMatrix, DenseMatrix64F tPowersVector, DenseMatrix64F polynomialCoefficientVector)
   {
      M1.reshape(identityMatrix.getNumRows(), identityMatrix.getNumCols());
      M1.zero();
      
      M2.reshape(alphaBetaPrimeMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
      M2.zero();
      
      M3.reshape(identityMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
      M3.zero();
      
      CommonOps.mult(gammaPrimeMatrix, backwardIterationMatrix, M1);
      CommonOps.subtract(identityMatrix, M1, M1);
      
      CommonOps.mult(terminalConstraintMatrix, tPowersVector, M2);
      CommonOps.mult(gammaPrimeMatrix, M2, M2);
      CommonOps.addEquals(M2, alphaBetaPrimeMatrix);
      
      CommonOps.invert(M1, M3);
      CommonOps.mult(M3, M2, M3);
      
      CommonOps.mult(M3, polynomialCoefficientVector, icpVector);
   }
   
   
   private void calculateAlphaPrimeMatrixOnSegment(DenseMatrix64F alphaPrimeMatrix, int segment, double time)
   {
      
   }
   
   private void calculateBetaPrimeMatrixOnSegment(DenseMatrix64F betaPrimeMatrix, int segment, double time)
   {
      
   }
   
   private void calculateGammaPrimeMatrixOnSegment(DenseMatrix64F gammaPrimeMatrix, int segment, double time)
   {
      
   }
   
   
   
   private void calculateAlphaPrimeOnCMPSegmentX(DenseMatrix64F alphaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrime, cmpPolynomialX, time);
   }
   
   private void calculateAlphaPrimeOnCMPSegmentY(DenseMatrix64F alphaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrime, cmpPolynomialY, time);
   }
   
   private void calculateAlphaPrimeOnCMPSegmentZ(DenseMatrix64F alphaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrime, cmpPolynomialZ, time);
   }
   
   private final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(10000, 1);
   private final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(10000, 1);
   
   private void calculateAlphaPrimeOnCMPSegment(DenseMatrix64F alphaPrime, YoPolynomial cmpPolynomial, double time)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i);
         CommonOps.addEquals(alphaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   
   
   private void calculateBetaPrimeOnCMPSegmentX(DenseMatrix64F betaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();
      
      double timeTrajectory = cmpPolynomialX.getXFinal();
      
      calculateBetaPrimeOnCMPSegment(betaPrime, cmpPolynomialX, time, timeTrajectory);
   }
   
   private void calculateBetaPrimeOnCMPSegmentY(DenseMatrix64F betaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();
      
      double timeTrajectory = cmpPolynomialY.getXFinal();
      
      calculateBetaPrimeOnCMPSegment(betaPrime, cmpPolynomialY, time, timeTrajectory);
   }
   
   private void calculateBetaPrimeOnCMPSegmentZ(DenseMatrix64F betaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      
      double timeTrajectory = cmpPolynomialZ.getXFinal();
      
      calculateBetaPrimeOnCMPSegment(betaPrime, cmpPolynomialZ, time, timeTrajectory);
   }
   
   private void calculateBetaPrimeOnCMPSegment(DenseMatrix64F betaPrime, YoPolynomial cmpPolynomial, double time, double timeTrajectory)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i) * Math.exp(omega0.getDoubleValue()*(time-timeTrajectory));
         CommonOps.addEquals(betaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateGammaPrimeOnCMPSegmentX(DenseMatrix64F gammaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      double timeTrajectory = cmpPolynomialX.getXFinal();
      
      calculateGammaPrimeOnCMPSegment(gammaPrime, time, timeTrajectory);     
   }
   
   private void calculateGammaPrimeOnCMPSegmentY(DenseMatrix64F gammaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();
      double timeTrajectory = cmpPolynomialY.getXFinal();
      
      calculateGammaPrimeOnCMPSegment(gammaPrime, time, timeTrajectory);    
   }
      
   private void calculateGammaPrimeOnCMPSegmentZ(DenseMatrix64F gammaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      
      double timeTrajectory = cmpPolynomialZ.getXFinal();
      
      calculateGammaPrimeOnCMPSegment(gammaPrime, time, timeTrajectory);
   }

   void calculateGammaPrimeOnCMPSegment(DenseMatrix64F gammaPrime, double time, double timeTrajectory)
   {
      gammaPrime.reshape(1, 1);
      gammaPrime.zero();
      
      double [] gamaPrimeValue = {Math.exp(omega0.getDoubleValue() * (time - timeTrajectory))};
      gammaPrime.setData(gamaPrimeValue);
   }  
      
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.set(desiredICPPositionOutput);
   }
   
   public void getVelocity(FrameVector velocityToPack)
   {
      
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
      
   }

   public void hideVisualization()
   {
      
   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }
}
