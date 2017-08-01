package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commonWalkingControlModules.dynamicReachability.SmoothCoMIntegrationTools;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothCapturePointTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * @author Tim Seyde
 */


public class ReferenceICPTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoDouble omega0;
   private ReferenceFrame trajectoryFrame;
   
   private final YoBoolean useDecoupled;
   
   private final static int FIRST_SEGMENT = 0;

   private final static int defaultSize = 1000;
   
   private final List<FramePoint> cmpDesiredFinalPositions = new ArrayList<>();

   private final List<FramePoint> icpDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint> icpDesiredFinalPositions = new ArrayList<>();
   
   private FramePoint icpPositionDesiredCurrent = new FramePoint();
   private FrameVector icpVelocityDesiredCurrent = new FrameVector();
   private FrameVector icpAccelerationDesiredCurrent = new FrameVector();
   private FrameVector icpVelocityDynamicsCurrent = new FrameVector();
   
   private final List<FramePoint> comDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint> comDesiredFinalPositions = new ArrayList<>();
   
   private FramePoint comPositionDesiredCurrent = new FramePoint();
   private FrameVector comVelocityDesiredCurrent = new FrameVector();
   private FrameVector comVelocityDynamicsCurrent = new FrameVector();
   
   private FramePoint icpPositionDesiredFinalFirstSegment = new FramePoint();
   private FramePoint comPositionDesiredFinalFirstSegment = new FramePoint();
   
   private FramePoint cmpPositionDesiredInitial = new FramePoint();
   private FramePoint icpPositionDesiredTerminal = new FramePoint();
   
   private YoFramePoint icpCurrentTest;
   private YoFramePoint icpFirstFinalTest;
   private YoFramePoint icpTerminalTest;
   private YoInteger cmpTrajectoryLength;
   private YoInteger icpTerminalLength;
   private YoInteger currentSegmentIndex;

   private final YoBoolean isStanding;
   private final YoBoolean areICPDynamicsSatisfied;
   private final YoBoolean areCoMDynamicsSatisfied;

   private final YoInteger totalNumberOfSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;
   
   private YoDouble startTimeOfCurrentPhase;
   private YoDouble localTimeInCurrentPhase;
   
   private boolean isPaused = false;
   
   private final List<YoFrameTrajectory3D> cmpTrajectories = new ArrayList<>();
   private final List<YoFrameTrajectory3D> icpTrajectories = new ArrayList<>();

   public ReferenceICPTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isStanding,
                                          YoBoolean useDecoupled, ReferenceFrame trajectoryFrame, YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.trajectoryFrame = trajectoryFrame;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isStanding = isStanding;
      this.useDecoupled = useDecoupled;
      
      areICPDynamicsSatisfied = new YoBoolean("areICPDynamicsSatisfied", registry);
      areICPDynamicsSatisfied.set(false);
      areCoMDynamicsSatisfied = new YoBoolean("areCoMDynamicsSatisfied", registry);
      areCoMDynamicsSatisfied.set(false);
      
      totalNumberOfSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);
      
      startTimeOfCurrentPhase = new YoDouble(namePrefix + "StartTimeCurrentPhase", registry);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "LocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);
      
      icpCurrentTest = new YoFramePoint("ICPCurrentTest", ReferenceFrame.getWorldFrame(), registry);
      icpFirstFinalTest = new YoFramePoint("ICPFirstFinalTest", ReferenceFrame.getWorldFrame(), registry);
      icpTerminalTest = new YoFramePoint("ICPTerminalTest", ReferenceFrame.getWorldFrame(), registry);
      cmpTrajectoryLength = new YoInteger("CMPTrajectoryLength", registry);
      icpTerminalLength = new YoInteger("ICPTerminalLength", registry);
      currentSegmentIndex = new YoInteger("CurrentSegment", registry);
      
      while(icpDesiredInitialPositions.size() < defaultSize)
      {
         icpDesiredInitialPositions.add(new FramePoint());
         icpDesiredFinalPositions.add(new FramePoint());
         cmpDesiredFinalPositions.add(new FramePoint());
         
         comDesiredInitialPositions.add(new FramePoint());
         comDesiredFinalPositions.add(new FramePoint());
      }
   }

   public void setNumberOfRegisteredSteps(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void reset()
   {
      cmpTrajectories.clear();
      icpTrajectories.clear();
      totalNumberOfSegments.set(0);
      localTimeInCurrentPhase.set(0.0);
   }

   public void initializeForTransfer(double initialTime, List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);
      
      PrintTools.debug("Transfer");

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
      
      initialize();
   }

   public void initializeForSwing(double initialTime, List<CMPTrajectory> transferCMPTrajectories, List<CMPTrajectory> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);
      
      PrintTools.debug("Swing");

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
      
      initialize();
   }

   @Override
   public void initialize()
   {
      // FIXME
//      if (!isStanding.getBooleanValue())
      {         
         if(useDecoupled.getBooleanValue())
         {
            SmoothCapturePointTools.computeDesiredCornerPointsDecoupled(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());
            
//            PrintTools.debug("Exit ICPs = " + icpDesiredFinalPositions.subList(0,17).toString());
//            PrintTools.debug("Entry ICPs = " + icpDesiredInitialPositions.subList(0,17).toString());
            SmoothCoMIntegrationTools.computeDesiredCenterOfMassCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, 
                                                                             comDesiredInitialPositions, comDesiredFinalPositions, 
                                                                             cmpTrajectories, omega0.getDoubleValue());

         }
         else
         {
            SmoothCapturePointTools.computeDesiredCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());
            
//            PrintTools.debug("Exit ICPs = " + icpDesiredFinalPositions.subList(0,17).toString());
//            PrintTools.debug("Entry ICPs = " + icpDesiredInitialPositions.subList(0,17).toString());
            SmoothCoMIntegrationTools.computeDesiredCenterOfMassCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, 
                                                                             comDesiredInitialPositions, comDesiredFinalPositions, 
                                                                             cmpTrajectories, omega0.getDoubleValue());
         }
         
         icpPositionDesiredTerminal.set(icpDesiredFinalPositions.get(cmpTrajectories.size() - 1));
         
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
         
         currentSegmentIndex.set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), cmpTrajectories));
         YoFrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(currentSegmentIndex.getIntegerValue());
         getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalFirstSegment, currentSegmentIndex.getIntegerValue());
         getCoMPositionDesiredFinalFromSegment(comPositionDesiredFinalFirstSegment, currentSegmentIndex.getIntegerValue());
         
         icpFirstFinalTest.set(icpPositionDesiredFinalFirstSegment);

         if(useDecoupled.getBooleanValue())
         {
            // ICP
            SmoothCapturePointTools.computeDesiredCapturePointPositionDecoupled(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                icpPositionDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointVelocityDecoupled(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                icpVelocityDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointAccelerationDecoupled(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                                    icpAccelerationDesiredCurrent);
            
            // CoM
            SmoothCoMIntegrationTools.computeDesiredCenterOfMassPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, comPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                         comPositionDesiredCurrent);
            SmoothCoMIntegrationTools.computeDesiredCenterOfMassVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, comPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                         comVelocityDesiredCurrent);
         }
         else
         {
            // ICP
            SmoothCapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpPositionDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                       icpVelocityDesiredCurrent);
            SmoothCapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                           icpAccelerationDesiredCurrent);
            
            // CoM
            SmoothCoMIntegrationTools.computeDesiredCenterOfMassPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, comPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                         comPositionDesiredCurrent);
            SmoothCoMIntegrationTools.computeDesiredCenterOfMassVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalFirstSegment, comPositionDesiredFinalFirstSegment, cmpPolynomial3D, 
                                                                         comVelocityDesiredCurrent);
         }
         checkICPDynamics(localTimeInCurrentPhase.getDoubleValue(), icpVelocityDesiredCurrent, icpPositionDesiredCurrent, cmpPolynomial3D);
         checkCoMDynamics(localTimeInCurrentPhase.getDoubleValue(), comVelocityDesiredCurrent, icpPositionDesiredCurrent, comPositionDesiredCurrent);
      }

      icpCurrentTest.set(icpPositionDesiredCurrent);
   }
   
   private void checkICPDynamics(double time, FrameVector icpVelocityDesiredCurrent, FramePoint icpPositionDesiredCurrent, YoFrameTrajectory3D cmpPolynomial3D)
   {
      cmpPolynomial3D.compute(time);
      
      icpVelocityDynamicsCurrent.sub(icpPositionDesiredCurrent, cmpPolynomial3D.getFramePosition());
      icpVelocityDynamicsCurrent.scale(omega0.getDoubleValue());
      
      areICPDynamicsSatisfied.set(icpVelocityDesiredCurrent.epsilonEquals(icpVelocityDynamicsCurrent, 10e-6));
      
//      PrintTools.debug("ICP velocity desired  = " + icpVelocityDesiredCurrent.toString());
//      PrintTools.debug("ICP velocity dynamics = " + icpVelocityDynamicsCurrent.toString());
   }
   
   private void checkCoMDynamics(double time, FrameVector comVelocityDesiredCurrent, FramePoint icpPositionDesiredCurrent, FramePoint comPositionDesiredCurrent)
   {      
      comVelocityDynamicsCurrent.sub(icpPositionDesiredCurrent, comPositionDesiredCurrent);
      comVelocityDynamicsCurrent.scale(omega0.getDoubleValue());
      
      areCoMDynamicsSatisfied.set(comVelocityDesiredCurrent.epsilonEquals(comVelocityDynamicsCurrent, 10e-6));
      
//      PrintTools.debug("CoM velocity desired  = " + comVelocityDesiredCurrent.toString());
//      PrintTools.debug("CoM velocity dynamics = " + comVelocityDynamicsCurrent.toString());
   }
   
   private int getCurrentSegmentIndex(double timeInCurrentPhase, List<YoFrameTrajectory3D> cmpTrajectories)
   {
      int currentSegmentIndex = FIRST_SEGMENT;
      while(timeInCurrentPhase > cmpTrajectories.get(currentSegmentIndex).getFinalTime() && Math.abs(cmpTrajectories.get(currentSegmentIndex).getFinalTime() - cmpTrajectories.get(currentSegmentIndex+1).getInitialTime()) < 1.0e-5)
      {
         currentSegmentIndex++;
      }
      return currentSegmentIndex;
   }
   
   public void getICPPositionDesiredFinalFromSegment(FramePoint icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment));
   }
   
   public void getCoMPositionDesiredFinalFromSegment(FramePoint comPositionDesiredFinal, int segment)
   {
      comPositionDesiredFinal.set(comDesiredFinalPositions.get(segment));
   }

   public void getCoMPosition(YoFramePoint comPositionToPack)
   {
      comPositionToPack.set(comPositionDesiredCurrent);
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }
   
   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }
   
   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredCurrent);
   }
   
   public void getAcceleration(YoFrameVector accelerationToPack)
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
   
   public void getLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
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
   
   public List<FramePoint> getCoMPositionDesiredInitialList()
   {
      return comDesiredInitialPositions;
   }
   
   public List<FramePoint> getCoMPositionDesiredFinalList()
   {
      return comDesiredFinalPositions;
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
