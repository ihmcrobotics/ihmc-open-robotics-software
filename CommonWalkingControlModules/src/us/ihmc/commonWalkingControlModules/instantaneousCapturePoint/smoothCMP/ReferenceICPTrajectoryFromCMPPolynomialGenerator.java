package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointMatrixTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceICPTrajectoryFromCMPPolynomialGenerator implements PositionTrajectoryGenerator
{
   private final YoDouble omega0;
   private ReferenceFrame trajectoryFrame;
   
   private final YoBoolean useDecoupled;
   
   private final static int FIRST_SEGMENT = 0;

   private final static int defaultSize = 1000;

   private final List<FramePoint> icpDesiredInitialPositionsNEW = new ArrayList<>();
   private final List<FramePoint> icpDesiredFinalPositionsNEW = new ArrayList<>();
   
   private final List<FramePoint> icpDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint> icpDesiredFinalPositions = new ArrayList<>();
   private final List<FramePoint> cmpDesiredFinalPositions = new ArrayList<>();
   
   private FramePoint icpPositionDesiredCurrent = new FramePoint();
   private FrameVector icpVelocityDesiredCurrent = new FrameVector();
   private FrameVector icpAccelerationDesiredCurrent = new FrameVector();
   
   private FramePoint icpPositionDesiredFinalFirstSegment = new FramePoint();
   
   private FramePoint cmpPositionDesiredInitial = new FramePoint();
   private FramePoint icpPositionDesiredTerminal = new FramePoint();

   double [] icpQuantityDesiredCurrent = new double[3];
   
   private DenseMatrix64F icpPositionDesiredInitialMatrix = new DenseMatrix64F(defaultSize, 3);
   private DenseMatrix64F icpPositionDesiredFinalMatrix = new DenseMatrix64F(defaultSize, 3);

   private final YoBoolean isStanding;

   private final YoInteger totalNumberOfSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;
   
   private double timeInCurrentSegment;
   
   private YoDouble timeCurrentPhase;

   private final List<YoFrameTrajectory3D> cmpTrajectories = new ArrayList<>();

   public ReferenceICPTrajectoryFromCMPPolynomialGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isStanding,
                                                           YoBoolean useDecoupled, ReferenceFrame trajectoryFrame, YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.trajectoryFrame = trajectoryFrame;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isStanding = isStanding;
      this.useDecoupled = useDecoupled;

      totalNumberOfSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);
      
      timeCurrentPhase = new YoDouble(namePrefix + "RelativeTimeCurrentPhase", registry);
      timeCurrentPhase.set(0.0);
      
      while(icpDesiredInitialPositions.size() < defaultSize)
      {
         icpDesiredInitialPositionsNEW.add(new FramePoint());
         icpDesiredFinalPositionsNEW.add(new FramePoint());
         
         icpDesiredInitialPositions.add(new FramePoint());
         icpDesiredFinalPositions.add(new FramePoint());
         cmpDesiredFinalPositions.add(new FramePoint());
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
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getPolynomials().get(cmpSegment));
            totalNumberOfSegments.increment();
         }

         CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getPolynomials().get(cmpSegment));
            totalNumberOfSegments.increment();
         }
      }

      CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getPolynomials().get(cmpSegment));
         totalNumberOfSegments.increment();
      }

      icpPositionDesiredInitialMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
      icpPositionDesiredFinalMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
   }

   public void initializeForSwing(List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();

      CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(0);
      int cmpSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(swingCMPTrajectory.getPolynomials().get(cmpSegment));
         totalNumberOfSegments.increment();
      }

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getPolynomials().get(cmpSegment));
            totalNumberOfSegments.increment();
         }

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getPolynomials().get(cmpSegment));
            totalNumberOfSegments.increment();
         }
      }

      CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getPolynomials().get(cmpSegment));
         totalNumberOfSegments.increment();
      }

      icpPositionDesiredInitialMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
      icpPositionDesiredFinalMatrix.reshape(totalNumberOfSegments.getIntegerValue(), 3);
   }

   @Override
   public void initialize()
   {
      if (!isStanding.getBooleanValue())
      {
         if(useDecoupled.getBooleanValue() == true)
         {
            CapturePointMatrixTools.computeDesiredCornerPointsDecoupled(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());
         }
         else
         {
            CapturePointMatrixTools.computeDesiredCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());
         }
         
         getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalFirstSegment, FIRST_SEGMENT);

      }
   }

   @Override
   public void compute(double time)
   {
      if (!isStanding.getBooleanValue())
      {
         initialize();
         
         timeInCurrentSegment = time; //TODO: use relative NOT absolute time
         
         YoFrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(FIRST_SEGMENT);

         if(useDecoupled.getBooleanValue() == true)
         {
            CapturePointMatrixTools.computeDesiredCapturePointPositionDecoupled(omega0.getDoubleValue(), timeCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                icpPositionDesiredCurrent);
            CapturePointMatrixTools.computeDesiredCapturePointVelocityDecoupled(omega0.getDoubleValue(), timeCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                icpVelocityDesiredCurrent);
            CapturePointMatrixTools.computeDesiredCapturePointAccelerationDecoupled(omega0.getDoubleValue(), timeCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                    icpAccelerationDesiredCurrent);
         }
         else
         {
            CapturePointMatrixTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpPositionDesiredCurrent);
            CapturePointMatrixTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpVelocityDesiredCurrent);
            CapturePointMatrixTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), timeCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpAccelerationDesiredCurrent);
         }

         timeCurrentPhase.set(timeCurrentPhase.getDoubleValue() + 0.006);
      }
   }
   
   public void getICPPositionDesiredFinalFromSegment(FramePoint icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment)); // for new boundary value calculation (wrong)
//      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(cmpTrajectories.size()-1-segment)); // for existing boundary value calculation
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
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredCurrent);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
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
   

   public List<FramePoint> getICPPositionDesiredInitialList()
   {
      return icpDesiredInitialPositions;
   }
   
   public List<FramePoint> getICPPositionDesiredFinalList()
   {
      return icpDesiredFinalPositions;
   }
   
   public List<FramePoint> getCMPPositionDesiredList()
   {
      for(int i = 0; i < cmpTrajectories.size(); i++)
      {
         YoTrajectory cmpPolynomialX = cmpTrajectories.get(i).getYoTrajectory(0);
         YoTrajectory cmpPolynomialY = cmpTrajectories.get(i).getYoTrajectory(1);
         YoTrajectory cmpPolynomialZ = cmpTrajectories.get(i).getYoTrajectory(2);
         
         cmpPolynomialX.compute(cmpPolynomialX.getFinalTime());
         cmpPolynomialY.compute(cmpPolynomialY.getFinalTime());
         cmpPolynomialZ.compute(cmpPolynomialZ.getFinalTime());
         
         FramePoint cmpPositionDesired = cmpDesiredFinalPositions.get(i);
         cmpPositionDesired.set(cmpPolynomialX.getPosition(), cmpPolynomialY.getPosition(), cmpPolynomialZ.getPosition());
      }
      return cmpDesiredFinalPositions;
   }
   
   public FramePoint getICPPositionDesiredTerminal()
   {
      return icpPositionDesiredTerminal;
   }
   
   public int getTotalNumberOfSegments()
   {
      return totalNumberOfSegments.getIntegerValue();
   }
}
