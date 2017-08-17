package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoTrajectory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceICPTrajectoryFromCMPPolynomialGenerator implements PositionTrajectoryGenerator
{
   private final YoDouble omega0;
   private ReferenceFrame trajectoryFrame;
   
   private final static int FIRST_SEGMENT = 0;
   private final static int POSITION = 0;
   private final static int VELOCITY = 1;
   private final static int ACCELERATION = 2;

   private final static int defaultSize = 1000;

   private final List<FramePoint3D> icpDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint3D> icpDesiredFinalPositions = new ArrayList<>();
   private final List<FramePoint3D> cmpDesiredFinalPositions = new ArrayList<>();
   
   private FramePoint3D icpPositionDesiredInitialBackwardIteration = new FramePoint3D();
   private FramePoint3D icpPositionDesiredFinalBackwardIteration = new FramePoint3D();
   
   private final DenseMatrix64F coefficientsCombinedVector = new DenseMatrix64F();
   private final DenseMatrix64F coefficientsCurrentVector = new DenseMatrix64F();
   
   private FramePoint3D icpPositionDesiredCurrent = new FramePoint3D();
   private FrameVector3D icpVelocityDesiredCurrent = new FrameVector3D();
   private FrameVector3D icpAccelerationDesiredCurrent = new FrameVector3D();
   
   private FramePoint3D icpPositionDesiredFinalFirstSegment = new FramePoint3D();
   
   private FramePoint3D cmpPositionDesiredInitial = new FramePoint3D();
   private FramePoint3D icpPositionDesiredTerminal = new FramePoint3D();

   double [] icpQuantityDesiredCurrent = new double[3];
   
   private YoDouble timeICPFinalSegmentInitial;
   private YoDouble timeICPFinalSegmentFinal;
   private YoDouble timeICPPenultimateSegmentInitial;
   private YoDouble timeICPPenultimateSegmentFinal;
   private YoDouble timeICPAntepenultimateSegmentInitial;
   private YoDouble timeICPAntepenultimateSegmentFinal;
   private YoDouble timeICPSegmentCurrent;
   
   private DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private DenseMatrix64F generalizedGammaPrimeMatrix = new DenseMatrix64F(1, 1);
   private DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);

   private DenseMatrix64F identityMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private DenseMatrix64F backwardIterationMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private DenseMatrix64F finalConstraintMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private DenseMatrix64F tPowersMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private DenseMatrix64F polynomialCoefficientVector = new DenseMatrix64F(defaultSize, defaultSize);
   
   private final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);

   
   private DenseMatrix64F icpPositionDesiredInitialMatrix = new DenseMatrix64F(defaultSize, 3);
   private DenseMatrix64F icpPositionDesiredFinalMatrix = new DenseMatrix64F(defaultSize, 3);
   
   // Pre-allocated helper matrices
   private final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F M2 = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F M3 = new DenseMatrix64F(defaultSize, defaultSize);

   private final YoBoolean isStanding;

   private final YoInteger totalNumberOfSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;
   
   private double timeInCurrentSegment;
   
   private YoDouble timeCurrentPhase;

   private final List<YoFrameTrajectory3D> cmpTrajectories = new ArrayList<>();

   public ReferenceICPTrajectoryFromCMPPolynomialGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isStanding,
                                                           ReferenceFrame trajectoryFrame, YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.trajectoryFrame = trajectoryFrame;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isStanding = isStanding;

      totalNumberOfSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);
      
      timeCurrentPhase = new YoDouble(namePrefix + "RelativeTimeCurrentPhase", registry);
      timeCurrentPhase.set(0.0);
      
      timeICPFinalSegmentInitial = new YoDouble(namePrefix + "InitialRelativeTimeOfLastICPSegment", registry);
      timeICPFinalSegmentFinal = new YoDouble(namePrefix + "FinalRelativeTimeOfLastICPSegment", registry);
      timeICPPenultimateSegmentInitial = new YoDouble(namePrefix + "InitialRelativeTimeOfPenultimateICPSegment", registry);
      timeICPPenultimateSegmentFinal = new YoDouble(namePrefix + "FinalRelativeTimeOfPenultimateICPSegment", registry);
      timeICPAntepenultimateSegmentInitial = new YoDouble(namePrefix + "InitialRelativeTimeOfAntepenultimateICPSegment", registry);
      timeICPAntepenultimateSegmentFinal = new YoDouble(namePrefix + "FinalRelativeTimeOfAntepenultimateICPSegment", registry);
      timeICPSegmentCurrent = new YoDouble(namePrefix + "CurrentRelativeTimeOfICPSegment", registry);
      
      while(icpDesiredInitialPositions.size() < defaultSize)
      {
         icpDesiredInitialPositions.add(new FramePoint3D());
         icpDesiredFinalPositions.add(new FramePoint3D());
         cmpDesiredFinalPositions.add(new FramePoint3D());
      }
   }

   public void setNumberOfRegisteredSteps(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void reset()
   {
      cmpTrajectories.clear();
      totalNumberOfSegments.set(0);
      timeCurrentPhase.set(0.0);
   }

   public void initializeForTransfer(List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
//      PrintTools.debug("Number of footsteps = " + numberOfSteps);
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfSegments.increment();
         }

         CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfSegments.increment();
         }
      }

      CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfSegments.increment();
      }

      icpPositionDesiredInitialMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
      icpPositionDesiredFinalMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
      
//      YoTrajectory cmpPolynomial = cmpTrajectories.get(0).getYoTrajectory(0);
//      PrintTools.debug("Number of segments (transfer) = " + totalNumberOfSegments.getIntegerValue());
//      PrintTools.debug("Segment time: (initial, final) = " + "(" + cmpPolynomial.getXInitial() + ", " + cmpPolynomial.getXFinal() + ")");
   }

   public void initializeForSwing(List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();

      CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(0);
      int cmpSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfSegments.increment();
      }

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
//      PrintTools.debug("Number of footsteps = " + numberOfSteps);
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfSegments.increment();
         }

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfSegments.increment();
         }
      }

      CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfSegments.increment();
      }

      icpPositionDesiredInitialMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
      icpPositionDesiredFinalMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
      
      YoTrajectory cmpPolynomial = cmpTrajectories.get(0).getYoTrajectory(0);
//      PrintTools.debug("Number of segments (swing) = " + totalNumberOfSegments.getIntegerValue());
//      PrintTools.debug("Segment time: (initial, final) = " + "(" + cmpPolynomial.getXInitial() + ", " + cmpPolynomial.getXFinal() + ")");
   }

   @Override
   public void initialize()
   {
      if (!isStanding.getBooleanValue())
      {
         calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar();
      }
   }

   @Override
   public void compute(double time)
   {
      if (!isStanding.getBooleanValue())
      {
         initialize();
         
         timeInCurrentSegment = time; //TODO: use relative NOT absolute time

         calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(icpPositionDesiredCurrent, POSITION, timeCurrentPhase.getDoubleValue());
         calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(icpVelocityDesiredCurrent, VELOCITY, timeCurrentPhase.getDoubleValue());
         calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(icpAccelerationDesiredCurrent, ACCELERATION, timeCurrentPhase.getDoubleValue());
         
         timeCurrentPhase.set(timeCurrentPhase.getDoubleValue() + 0.006);
         if(timeCurrentPhase.getDoubleValue() > 0.8)
         {
            timeCurrentPhase.set(0.8);
         }
      }
   }

   public void getDesiredICP(YoFramePoint desiredICPToPack) //TODO: not sure whether this one is needed
   {
      desiredICPToPack.set(icpPositionDesiredCurrent);
   }
   
   public void getDesiredICPFinalFirstSegment(YoFramePoint desiredICPFinalToPack) //TODO: not sure whether this one is needed
   {
      desiredICPFinalToPack.set(icpPositionDesiredFinalFirstSegment);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredCurrent);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {

   }

   @Override
   public void hideVisualization()
   {

   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

   // SCALAR FORMULATION
   /**
    * Compute the i-th derivative of the reference ICP at the current time: &xi;<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>)
    * @param icpQuantityDesiredOutput
    * @param icpDerivativeOrder
    * @param time
    */
   private void calculateICPQuantityDesiredCurrentFromCMPPolynomialsScalar(FrameTuple3D<?, ?> icpQuantityDesiredOutput, int icpDerivativeOrder, double time)
   {            
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpTrajectories.get(FIRST_SEGMENT).getYoTrajectory(dir.ordinal());
         double icpPositionDesiredFinal = icpPositionDesiredFinalMatrix.get(FIRST_SEGMENT, dir.ordinal());
         
         icpQuantityDesiredCurrent[dir.ordinal()] = calculateICPQuantityFromCorrespondingCMPPolynomialScalar(icpDerivativeOrder, cmpPolynomial, icpPositionDesiredFinal, time);
      }
      icpQuantityDesiredOutput.setIncludingFrame(trajectoryFrame, icpQuantityDesiredCurrent);
   }
   
   /**
    * Backward iteration to determine &xi;<sub>ref,&phi;</sub>(0) and &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>) for all segments &phi;
    */
   private void calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar()
   {       
      setICPTerminalConditionScalar();
      
//      PrintTools.debug("Number of segments = " + totalNumberOfSegments.getIntegerValue());
      
      for(int i = totalNumberOfSegments.getIntegerValue() - 1; i >= 0; i--)
      {
         for(Direction dir : Direction.values())
         {
            YoTrajectory cmpPolynomial = cmpTrajectories.get(i).getYoTrajectory(dir.ordinal());
            double icpPositionDesiredFinal = icpPositionDesiredFinalMatrix.get(i, dir.ordinal());
            double time = cmpPolynomial.getInitialTime();
                        
            double icpPositionDesiredInitial = calculateICPQuantityFromCorrespondingCMPPolynomialScalar(POSITION, cmpPolynomial, icpPositionDesiredFinal, time);
            icpPositionDesiredInitialMatrix.set(i, dir.ordinal(), icpPositionDesiredInitial);
            
            if(i > 0)
            {
               icpPositionDesiredFinalMatrix.set(i-1, dir.ordinal(), icpPositionDesiredInitial);
            }
         }
         
         if(i == totalNumberOfSegments.getIntegerValue() - 1)
         {
            YoTrajectory cmpPolynomial = cmpTrajectories.get(i).getYoTrajectory(0);
            timeICPFinalSegmentInitial.set(cmpPolynomial.getInitialTime());
            timeICPFinalSegmentFinal.set(cmpPolynomial.getFinalTime());
//            PrintTools.debug("Time (intial, final) = " + "(" + cmpPolynomial.getXInitial() + ", " + cmpPolynomial.getXFinal() + ")");
         }
         if(i == totalNumberOfSegments.getIntegerValue() - 2)
         {
            YoTrajectory cmpPolynomial = cmpTrajectories.get(i).getYoTrajectory(0);
            timeICPPenultimateSegmentInitial.set(cmpPolynomial.getInitialTime());
            timeICPPenultimateSegmentFinal.set(cmpPolynomial.getFinalTime());
//            PrintTools.debug("Time (intial, final) = " + "(" + cmpPolynomial.getXInitial() + ", " + cmpPolynomial.getXFinal() + ")");
         }
         if(i == totalNumberOfSegments.getIntegerValue() - 3)
         {
            YoTrajectory cmpPolynomial = cmpTrajectories.get(i).getYoTrajectory(0);
            timeICPAntepenultimateSegmentInitial.set(cmpPolynomial.getInitialTime());
            timeICPAntepenultimateSegmentFinal.set(cmpPolynomial.getFinalTime());
//            PrintTools.debug("Time (intial, final) = " + "(" + cmpPolynomial.getXInitial() + ", " + cmpPolynomial.getXFinal() + ")");
         }
         
         icpPositionDesiredInitialBackwardIteration.set(icpPositionDesiredInitialMatrix.get(i, Direction.X.ordinal()), 
                                                        icpPositionDesiredInitialMatrix.get(i, Direction.Y.ordinal()), 
                                                        icpPositionDesiredInitialMatrix.get(i, Direction.Z.ordinal()));
         FramePoint3D icpPositionDesiredInitialBWI = icpDesiredInitialPositions.get(totalNumberOfSegments.getIntegerValue() - 1 - i);
         icpPositionDesiredInitialBWI.set(icpPositionDesiredInitialBackwardIteration.getX(), icpPositionDesiredInitialBackwardIteration.getY(), icpPositionDesiredInitialBackwardIteration.getZ());
         
         icpPositionDesiredFinalBackwardIteration.set(icpPositionDesiredFinalMatrix.get(i, Direction.X.ordinal()), 
                                                      icpPositionDesiredFinalMatrix.get(i, Direction.Y.ordinal()), 
                                                      icpPositionDesiredFinalMatrix.get(i, Direction.Z.ordinal()));
         FramePoint3D icpPositionDesiredFinalBWI = icpDesiredFinalPositions.get(totalNumberOfSegments.getIntegerValue() - 1 - i);
         icpPositionDesiredFinalBWI.set(icpPositionDesiredFinalBackwardIteration.getX(), icpPositionDesiredFinalBackwardIteration.getY(), icpPositionDesiredFinalBackwardIteration.getZ());
      }

      icpPositionDesiredFinalFirstSegment.set(icpPositionDesiredFinalMatrix.get(FIRST_SEGMENT, Direction.X.ordinal()), 
                                              icpPositionDesiredFinalMatrix.get(FIRST_SEGMENT, Direction.Y.ordinal()), 
                                              icpPositionDesiredFinalMatrix.get(FIRST_SEGMENT, Direction.Z.ordinal()));
   }
   
   /**
    * Setting the terminal DCM equal to corresponding CMP to initialize the DCM backward iteration
    * <P>
    * &xi;<sub>ref,T,n<sub>&phi;</sub></sub> = &nu;<sub>ref,T,n<sub>&phi;</sub></sub>
    * 
    */
   private void setICPTerminalConditionScalar()
   {
      for(int i = 0; i < Direction.size; i++)
      {
         YoTrajectory cmpPolynomialFinalSegment = cmpTrajectories.get(totalNumberOfSegments.getIntegerValue() - 1).getYoTrajectory(i);
         cmpPolynomialFinalSegment.compute(cmpPolynomialFinalSegment.getFinalTime());
         icpPositionDesiredFinalMatrix.set(totalNumberOfSegments.getIntegerValue() - 1, i, cmpPolynomialFinalSegment.getPosition());
      }
      icpPositionDesiredTerminal.set(icpPositionDesiredFinalMatrix.get(totalNumberOfSegments.getIntegerValue() - 1, 0),
                                     icpPositionDesiredFinalMatrix.get(totalNumberOfSegments.getIntegerValue() - 1, 1),
                                     icpPositionDesiredFinalMatrix.get(totalNumberOfSegments.getIntegerValue() - 1, 2));
//      PrintTools.debug("Final ICP = (" + icpPositionDesiredFinalMatrix.get(totalNumberOfSegments.getIntegerValue() - 1, 0) + ", "
//                                       + icpPositionDesiredFinalMatrix.get(totalNumberOfSegments.getIntegerValue() - 1, 1) + ", "
//                                       + icpPositionDesiredFinalMatrix.get(totalNumberOfSegments.getIntegerValue() - 1, 2) 
//                                       + ")");
   }
   
   /**
    * Variation of J. Englsberger's "Smooth trajectory generation and push-recovery based on DCM"
    * <br>
    * The approach for calculating DCMs is based on CMP polynomials instead of discrete waypoints
    * 
    * @param icpDerivativeOrder
    * @param cmpPolynomial
    * @param icpPositionDesiredFinal
    * @param time
    * @return
    */
   private double calculateICPQuantityFromCorrespondingCMPPolynomialScalar(int icpDerivativeOrder, YoTrajectory cmpPolynomial, double icpPositionDesiredFinal, double time)
   {  
      timeICPSegmentCurrent.set(time);
      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();
      polynomialCoefficientVector.reshape(numberOfCoefficients, 1);
      polynomialCoefficientVector.setData(polynomialCoefficients);
      
      generalizedAlphaPrimeMatrix.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeMatrix.reshape(1, numberOfCoefficients);
      generalizedAlphaBetaPrimeMatrix.reshape(1, numberOfCoefficients);
      generalizedGammaPrimeMatrix.reshape(1, 1);

      generalizedAlphaPrimeMatrix.zero();
      generalizedBetaPrimeMatrix.zero();
      generalizedAlphaBetaPrimeMatrix.zero();
      generalizedGammaPrimeMatrix.zero();

      calculateGeneralizedAlphaPrimeOnCMPSegment(generalizedAlphaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
      calculateGeneralizedBetaPrimeOnCMPSegment(generalizedBetaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
      calculateGeneralizedGammaPrimeOnCMPSegment(generalizedGammaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);
      
//      PrintTools.debug("A: " +generalizedAlphaPrimeMatrix.toString());
//      PrintTools.debug("B: " +generalizedBetaPrimeMatrix.toString());
//      PrintTools.debug("C: " +generalizedGammaPrimeMatrix.toString());
//      PrintTools.debug("AB: " + generalizedAlphaBetaPrimeMatrix.toString());
//      PrintTools.debug("P: " + polynomialCoefficientVector.toString());
//      PrintTools.debug("ICP: " + icpPositionDesiredFinal);
//      PrintTools.debug("");
      
      return calculateICPQuantityScalar(generalizedAlphaBetaPrimeMatrix, generalizedGammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
   }
   
   /**
    * Compute the i-th derivative of &xi;<sub>ref,&phi;</sub> at time t<sub>&phi;</sub>: 
    * <P>
    * &xi;<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>) = 
    * (&alpha;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>)
    *  - &beta;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub>
    *  + &gamma;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) * &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaBetaPrimeMatrix
    * @param generalizedGammaPrimeMatrix
    * @param polynomialCoefficientVector
    * @param icpPositionDesiredFinal
    * @return
    */
   private double calculateICPQuantityScalar(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
                                             DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal)
   {
      //fixme yeah this is wrong, TODO: why?
      M1.reshape(generalizedAlphaBetaPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientVector, M1);

      M2.set(generalizedGammaPrimeMatrix);
      CommonOps.scale(icpPositionDesiredFinal, M2);
      
      CommonOps.addEquals(M1, M2);
      
      return M1.get(0, 0);
   }
   
   /**
    * Compute the i-th derivative of &alpha;<sub>&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &alpha;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-j</sup> *
    * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaPrime
    * @param alphaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   private void calculateGeneralizedAlphaPrimeOnCMPSegment(DenseMatrix64F generalizedAlphaPrime, int alphaDerivativeOrder, YoTrajectory cmpPolynomial, double time)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);

      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();
         
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + alphaDerivativeOrder, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0.getDoubleValue(), -i);
         CommonOps.addEquals(generalizedAlphaPrime, scalar, tPowersDerivativeVectorTranspose);
         
//         PrintTools.debug("A = " + generalizedAlphaPrime .toString());
      }
   }
   
   /**
    * Compute the i-th derivative of &beta;<sub>&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &beta;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-(j-i)</sup> *
    * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param generalizedBetaPrime
    * @param betaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   private void calculateGeneralizedBetaPrimeOnCMPSegment(DenseMatrix64F generalizedBetaPrime, int betaDerivativeOrder, YoTrajectory cmpPolynomial, double time)
   {            
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);

      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();

         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, timeSegmentTotal));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0.getDoubleValue(), betaDerivativeOrder-i) * Math.exp(omega0.getDoubleValue()*(time-timeSegmentTotal));
         CommonOps.addEquals(generalizedBetaPrime, scalar, tPowersDerivativeVectorTranspose);
         
//         PrintTools.debug("B = " + generalizedBetaPrime.toString());
      }
   }

   /**
    * Compute the i-th derivative of &gamma;<sub>&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) = &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param generalizedGammaPrime
    * @param gammaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   private void calculateGeneralizedGammaPrimeOnCMPSegment(DenseMatrix64F generalizedGammaPrime, int gammaDerivativeOrder, YoTrajectory cmpPolynomial, double time)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddGamaPrimeValue = Math.pow(omega0.getDoubleValue(), gammaDerivativeOrder)*Math.exp(omega0.getDoubleValue() * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
      
//      PrintTools.debug("C = " + generalizedGammaPrime.toString());
   } 
   
   
   public List<FramePoint3D> getICPPositionDesiredInitialList()
   {
//      PrintTools.debug("Size = " + icpDesiredInitialPositions.size());
      return icpDesiredInitialPositions;
   }
   
   public List<FramePoint3D> getICPPositionDesiredFinalList()
   {
//      PrintTools.debug("Size = " + icpDesiredFinalPositions.size());
      return icpDesiredFinalPositions;
   }
   
   public List<FramePoint3D> getCMPPositionDesiredList()
   {
      for(int i = 0; i < cmpTrajectories.size(); i++)
      {
         YoTrajectory cmpPolynomialX = cmpTrajectories.get(i).getYoTrajectory(0);
         YoTrajectory cmpPolynomialY = cmpTrajectories.get(i).getYoTrajectory(1);
         YoTrajectory cmpPolynomialZ = cmpTrajectories.get(i).getYoTrajectory(2);
         
         cmpPolynomialX.compute(cmpPolynomialX.getFinalTime());
         cmpPolynomialY.compute(cmpPolynomialY.getFinalTime());
         cmpPolynomialZ.compute(cmpPolynomialZ.getFinalTime());
         
         FramePoint3D cmpPositionDesired = cmpDesiredFinalPositions.get(i);
         cmpPositionDesired.set(cmpPolynomialX.getPosition(), cmpPolynomialY.getPosition(), cmpPolynomialZ.getPosition());
      }
      return cmpDesiredFinalPositions;
   }
   
   public FramePoint3D getICPPositionDesiredTerminal()
   {
      return icpPositionDesiredTerminal;
   }
   
   public int getTotalNumberOfSegments()
   {
      return totalNumberOfSegments.getIntegerValue();
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
//      double desiredICPX = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getX(), cmpPolynomialTrajectory.getYoTrajectoryX(), time, trajectoryTime);
//      double desiredICPY = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getY(), cmpPolynomialTrajectory.getYoTrajectoryY(), time, trajectoryTime);
//      double desiredICPZ = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getZ(), cmpPolynomialTrajectory.getYoTrajectoryZ(), time, trajectoryTime);
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
