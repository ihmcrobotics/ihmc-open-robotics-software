package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPPlannerTrajectoryFromCMPPolynomialGenerator implements PositionTrajectoryGenerator
{
   private final DoubleYoVariable omega0;
   private ReferenceFrame trajectoryFrame;
   
   private final static int firstSegment = 0;
   private final static int POSITION = 0;
   private final static int VELOCITY = 1;
   private final static int ACCELERATION = 2;
   

   private final List<YoPolynomial3D> cmpPolynomialTrajectories = new ArrayList<>();
   
   private final List<FramePoint> desiredICPBoundaryPositions = new ArrayList<>();
   
   private final FramePoint desiredICPPositionOutput = new FramePoint();
   private final FrameVector desiredICPVelocityOutput = new FrameVector();
   
   private final DenseMatrix64F coefficientsCombinedVector = new DenseMatrix64F();
   private final DenseMatrix64F coefficientsCurrentVector = new DenseMatrix64F();
   
   FramePoint icpPositionDesiredCurrent = new FramePoint();
   FrameVector icpVelocityDesiredCurrent = new FrameVector();
   FrameVector icpAccelerationDesiredCurrent = new FrameVector();
   
   FramePoint icpPositionDesiredFinal = new FramePoint();

   double [] icpQuantityDesiredCurrent = new double[3];
   
   private DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F generalizedGammaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F();

   private DenseMatrix64F identityMatrix = new DenseMatrix64F();
   private DenseMatrix64F backwardIterationMatrix = new DenseMatrix64F();
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
      calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(icpPositionDesiredCurrent, POSITION, time);
      calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(icpVelocityDesiredCurrent, VELOCITY, time);
      calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(icpAccelerationDesiredCurrent, ACCELERATION, time);
   }

   // SCALAR FORMULATION
   private void calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(FrameTuple<?, ?> icpQuantityDesiredOutput, int icpDerivativeOrder, double time)
   {
      for(Direction dir : Direction.values())
      {
         YoPolynomial cmpPolynomial = cmpPolynomialTrajectories.get(firstSegment).getYoPolynomial(dir.ordinal());
         icpQuantityDesiredCurrent[dir.ordinal()] = calculateICPQuantityFromCorrespondingCMPPolynomialScalar(icpDerivativeOrder, cmpPolynomial, icpPositionDesiredFinalMatrix.get(firstSegment, dir.ordinal()), time);
      }
      icpQuantityDesiredOutput.setIncludingFrame(trajectoryFrame, icpQuantityDesiredCurrent);
   }

   private void calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar()
   {       
      setICPTerminalConditionScalar();
      
      for(int i = numberOfSegments-1; i >= 0; i--)
      {
         for(Direction dir : Direction.values())
         {
            YoPolynomial cmpPolynomial = cmpPolynomialTrajectories.get(i).getYoPolynomial(dir.ordinal());
            double icpPositionDesiredInitial = calculateICPQuantityFromCorrespondingCMPPolynomialScalar(POSITION, cmpPolynomial, icpPositionDesiredFinalMatrix.get(i, dir.ordinal()), 0.0);
            
            icpPositionDesiredInitialMatrix.set(i, dir.ordinal(), icpPositionDesiredInitial);
            
            if(i > 0)
            {
               icpPositionDesiredFinalMatrix.set(i-1, dir.ordinal(), icpPositionDesiredInitial);
               
            }
         }
      }
      icpPositionDesiredFinal.set(icpPositionDesiredFinalMatrix.get(firstSegment, Direction.X.ordinal()), 
                                  icpPositionDesiredFinalMatrix.get(firstSegment, Direction.Y.ordinal()), 
                                  icpPositionDesiredFinalMatrix.get(firstSegment, Direction.Z.ordinal()));
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
   
   
   private double calculateICPQuantityFromCorrespondingCMPPolynomialScalar(int icpDerivativeOrder, YoPolynomial cmpPolynomial, double icpPositionDesiredFinal, double time)
   {            
      DenseMatrix64F polynomialCoefficientVector =  cmpPolynomial.getCoefficientsVector();
      
      calculateGeneralizedAlphaPrimeOnCMPSegment(generalizedAlphaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
      calculateGeneralizedBetaPrimeOnCMPSegment(generalizedBetaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
      calculateGeneralizedGammaPrimeOnCMPSegment(generalizedGammaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);
      
      return calculateICPQuantityScalar(generalizedAlphaBetaPrimeMatrix, generalizedGammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
   }
   
   private double calculateICPQuantityScalar(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
                                             DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal)
   {
      double icpQuantityDesired = 0.0;
      
      M1.reshape(generalizedAlphaBetaPrimeMatrix.getNumRows(), generalizedAlphaBetaPrimeMatrix.getNumCols());
      M1.zero();
      M2.reshape(generalizedGammaPrimeMatrix.getNumRows(), generalizedGammaPrimeMatrix.getNumCols());
      M2.zero();
      
      CommonOps.mult(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientVector, M1);
      CommonOps.scale(icpPositionDesiredFinal, generalizedGammaPrimeMatrix, M2);
      
      CommonOps.addEquals(M1, M2);
      
      icpQuantityDesired = M1.get(0, 0);
      return icpQuantityDesired;
   }

   
   private void calculateGeneralizedAlphaPrimeOnCMPSegment(DenseMatrix64F generalizedAlphaPrime, int alphaDerivativeOrder, YoPolynomial cmpPolynomial, double time)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i+alphaDerivativeOrder, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i);
         CommonOps.addEquals(generalizedAlphaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateGeneralizedBetaPrimeOnCMPSegment(DenseMatrix64F generalizedBetaPrime, int betaDerivativeOrder, YoPolynomial cmpPolynomial, double time)
   {            
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeTrajectory = cmpPolynomial.getXFinal();

      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i-betaDerivativeOrder) * Math.exp(omega0.getDoubleValue()*(time-timeTrajectory));
         CommonOps.addEquals(generalizedBetaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }

   void calculateGeneralizedGammaPrimeOnCMPSegment(DenseMatrix64F generalizedGammaPrime, int gammaDerivativeOrder, YoPolynomial cmpPolynomial, double time)
   {      
      double timeTrajectory = cmpPolynomial.getXFinal();
      
      generalizedGammaPrime.reshape(1, 1);
      generalizedGammaPrime.zero();
      
      double [] ddGamaPrimeValue = {Math.pow(omega0.getDoubleValue(), gammaDerivativeOrder)*Math.exp(omega0.getDoubleValue() * (time - timeTrajectory))};
      generalizedGammaPrime.setData(ddGamaPrimeValue);
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
   
   // TODO: FIRST IMPLEMENTATION --> check whether still usable
// desiredICPCurrent: desired ICP at time; modified.
// TODO: take care of frame matching
//   private void calculateICPFromCorrespondingCMPPolynomial(YoFramePoint desiredICPCurrent, YoFramePoint desiredICPFinal, YoPolynomial3D cmpPolynomialTrajectory, double time, double trajectoryTime)
//   {
//      Point3D desiredICPCurrentPosition = new Point3D();
//      desiredICPCurrent.getPoint(desiredICPCurrentPosition);
//      
//      Point3D desiredICPFinalPosition = new Point3D();
//      desiredICPFinal.getPoint(desiredICPFinalPosition);
//      
//      calculateICPFromCorrespondingCMPPolynomial(desiredICPCurrentPosition, desiredICPFinalPosition, cmpPolynomialTrajectory, time, trajectoryTime);
//      
//      desiredICPCurrent.setPoint(desiredICPCurrentPosition);
//   }
//   
//   private void calculateICPFromCorrespondingCMPPolynomial(Point3D desiredICPCurrent, Point3D desiredICPFinal, YoPolynomial3D cmpPolynomialTrajectory, double time, double trajectoryTime)
//   {     
//      double desiredICPX = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getX(), cmpPolynomialTrajectory.getYoPolynomialX(), time, trajectoryTime);
//      double desiredICPY = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getY(), cmpPolynomialTrajectory.getYoPolynomialY(), time, trajectoryTime);
//      double desiredICPZ = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getZ(), cmpPolynomialTrajectory.getYoPolynomialZ(), time, trajectoryTime);
//      
//      desiredICPCurrent.set(new Point3D(desiredICPX, desiredICPY, desiredICPZ));
//   }
//   
//   private double calculateICPFromCorrespondingCMPPolynomial1D(double desiredICPFinalPosition1D, YoPolynomial cmpPolynomialTrajectory1D, double time, double trajectoryTime)
//   {      
//      double exponentialFactor = Math.exp(omega0.getDoubleValue()*(time-trajectoryTime));
//      
//      double desiredICPCurrentPosition1D = desiredICPFinalPosition1D * exponentialFactor;  
//      for(int i = 0; i < cmpPolynomialTrajectory1D.getNumberOfCoefficients(); ++i)
//      {
//         desiredICPCurrentPosition1D += 1/Math.pow(omega0.getDoubleValue(), i) * (cmpPolynomialTrajectory1D.getDerivative(i, time) - cmpPolynomialTrajectory1D.getDerivative(i, trajectoryTime) * exponentialFactor);
//      }
//      
//      return desiredICPCurrentPosition1D;
//   }
}
