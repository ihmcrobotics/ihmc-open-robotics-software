package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothCapturePointTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceICPTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoDouble omega0;
   private ReferenceFrame trajectoryFrame;
   
   private final YoBoolean useDecoupled;
   
   private final static int FIRST_SEGMENT = 0;

   private final static int defaultSize = 1000;
   
   private final List<FramePoint> icpDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint> icpDesiredFinalPositions = new ArrayList<>();
   private final List<FramePoint> cmpDesiredFinalPositions = new ArrayList<>();
   
   private FramePoint icpPositionDesiredCurrent = new FramePoint();
   private FrameVector icpVelocityDesiredCurrent = new FrameVector();
   private FrameVector icpAccelerationDesiredCurrent = new FrameVector();
   
   private FramePoint icpPositionDesiredFinalFirstSegment = new FramePoint();
   
   private FramePoint cmpPositionDesiredInitial = new FramePoint();
   private FramePoint icpPositionDesiredTerminal = new FramePoint();
   
   private YoFramePoint icpTerminalTest;
   private YoInteger cmpTrajectoryLength;
   private YoInteger icpTerminalLength;

   double [] icpQuantityDesiredCurrent = new double[3];

   private final YoBoolean isStanding;

   private final YoInteger totalNumberOfSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;
   
   private YoDouble startTimeOfCurrentPhase;
   private YoDouble localTimeInCurrentPhase;
   private YoDouble durationOfPreviousPhase;
   
   private boolean isPaused = false;
   
   private final List<YoFrameTrajectory3D> cmpTrajectories = new ArrayList<>();

   public ReferenceICPTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isStanding,
                                          YoBoolean useDecoupled, ReferenceFrame trajectoryFrame, YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.trajectoryFrame = trajectoryFrame;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isStanding = isStanding;
      this.useDecoupled = useDecoupled;
      
      totalNumberOfSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);
      
      startTimeOfCurrentPhase = new YoDouble(namePrefix + "StartTimeCurrentPhase", registry);
      startTimeOfCurrentPhase.set(0.0);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "LocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);
      durationOfPreviousPhase = new YoDouble(namePrefix + "DurationPreviousPhase", registry);
      durationOfPreviousPhase.set(0.0);
      
      icpTerminalTest = new YoFramePoint("ICPTerminalTest", ReferenceFrame.getWorldFrame(), registry);
      cmpTrajectoryLength = new YoInteger("CMPTrajectoryLength", registry);
      icpTerminalLength = new YoInteger("ICPTerminalLength", registry);
      
      while(icpDesiredInitialPositions.size() < defaultSize)
      {
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
      localTimeInCurrentPhase.set(0.0);
   }

   public void initializeForTransfer(List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.add(durationOfPreviousPhase.getDoubleValue());

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
         
         durationOfPreviousPhase.set(transferCMPTrajectories.get(0).getPolynomials().get(cmpSegments-1).getFinalTime());

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
      
      initialize();
   }

   public void initializeForSwing(List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.add(durationOfPreviousPhase.getDoubleValue());

      CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(0);
      int cmpSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(swingCMPTrajectory.getPolynomials().get(cmpSegment));
         totalNumberOfSegments.increment();
      }
      
      durationOfPreviousPhase.set(swingCMPTrajectories.get(0).getPolynomials().get(cmpSegments-1).getFinalTime());
      
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
      
      initialize();
   }

   @Override
   public void initialize()
   {
      if (!isStanding.getBooleanValue())
      {
         if(useDecoupled.getBooleanValue())
         {
            SmoothCapturePointTools.computeDesiredCornerPointsDecoupled(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());
         }
         else
         {
            SmoothCapturePointTools.computeDesiredCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());
         }
         icpPositionDesiredTerminal.set(icpDesiredFinalPositions.get(cmpTrajectories.size() - 1));
         getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalFirstSegment, FIRST_SEGMENT);
         
         icpTerminalTest.set(icpPositionDesiredTerminal);
         cmpTrajectoryLength.set(cmpTrajectories.size());
         icpTerminalLength.set(icpDesiredFinalPositions.size());
      }
   }

   @Override
   public void compute(double time)
   {
      if (!isStanding.getBooleanValue())
      {
         localTimeInCurrentPhase.set(time - startTimeOfCurrentPhase.getDoubleValue());
         
         YoFrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(FIRST_SEGMENT);

         if(useDecoupled.getBooleanValue())
         {
            SmoothCapturePointTools.computeDesiredCapturePointPositionDecoupled(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                icpPositionDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointVelocityDecoupled(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                icpVelocityDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointAccelerationDecoupled(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                    icpAccelerationDesiredCurrent);
         }
         else
         {
            SmoothCapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpPositionDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpVelocityDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpAccelerationDesiredCurrent);
         }
      }
   }
   
   public void getICPPositionDesiredFinalFromSegment(FramePoint icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment));
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
   
   public void pause()
   {
      isPaused = true;
   }
   
   public void resume()
   {
      isPaused = false;
   }
}
