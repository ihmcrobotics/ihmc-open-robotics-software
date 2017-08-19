package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.dynamicReachability.SmoothCoMIntegrationToolbox;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothCapturePointAdjustmentToolbox;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothCapturePointToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * @author Tim Seyde
 */

public class ReferenceICPTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static boolean CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY = true;

   private final YoDouble omega0;
   private ReferenceFrame trajectoryFrame;

   private final static int FIRST_SEGMENT = 0;

   private final static int defaultSize = 1000;

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<FramePoint3D> cmpDesiredFinalPositions = new ArrayList<>();

   private final List<FramePoint3D> icpDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint3D> icpDesiredFinalPositions = new ArrayList<>();
   
   private final List<FramePoint3D> icpDesiredInitialPositionsFromCoPs = new ArrayList<>();
   private final List<FramePoint3D> icpDesiredFinalPositionsFromCoPs = new ArrayList<>();

   private FramePoint3D icpPositionDesiredCurrent = new FramePoint3D();
   private FrameVector3D icpVelocityDesiredCurrent = new FrameVector3D();
   private FrameVector3D icpAccelerationDesiredCurrent = new FrameVector3D();
   private FrameVector3D icpVelocityDynamicsCurrent = new FrameVector3D();
   private YoFrameVector yoICPVelocityDynamicsCurrent;

   private final List<FramePoint3D> comDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint3D> comDesiredFinalPositions = new ArrayList<>();

   private FramePoint3D comPositionDesiredCurrent = new FramePoint3D();
   private FrameVector3D comVelocityDesiredCurrent = new FrameVector3D();
   private FrameVector3D comAccelerationDesiredCurrent = new FrameVector3D();
   private FrameVector3D comVelocityDynamicsCurrent = new FrameVector3D();

   private FramePoint3D icpPositionDesiredFinalCurrentSegment = new FramePoint3D();
   private FramePoint3D comPositionDesiredInitialCurrentSegment = new FramePoint3D();
   private FramePoint3D comPositionDesiredFinalCurrentSegment = new FramePoint3D();

   private FramePoint3D cmpPositionDesiredInitial = new FramePoint3D();
   private FramePoint3D icpPositionDesiredTerminal = new FramePoint3D();
   
   private YoInteger cmpTrajectoryLength;
   private YoInteger icpTerminalLength;
   private YoInteger currentSegmentIndex;

   private final YoBoolean isStanding;
   private final YoBoolean isInitialTransfer;
   private final YoBoolean isDoubleSupport;

   private final YoBoolean areICPDynamicsSatisfied;
   private final YoBoolean areCoMDynamicsSatisfied;

   private final YoInteger totalNumberOfCMPSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;
   
   private int numberOfSegmentsSwing0;
   private int numberOfSegmentsTransfer0;

   private YoDouble startTimeOfCurrentPhase;
   private YoDouble localTimeInCurrentPhase;

   private boolean isPaused = false;

   private final List<FrameTrajectory3D> copTrajectories = new ArrayList<>();
   private final List<FrameTrajectory3D> cmpTrajectories = new ArrayList<>();

   private final YoBoolean continuouslyAdjustForICPContinuity;
   private List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList = new ArrayList<FrameTuple3D<?, ?>>();

   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final SmoothCoMIntegrationToolbox comToolbox = new SmoothCoMIntegrationToolbox(icpToolbox);
   private final SmoothCapturePointAdjustmentToolbox icpAdjustmentToolbox = new SmoothCapturePointAdjustmentToolbox(icpToolbox);

   public ReferenceICPTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isStanding,
                                          YoBoolean isInitialTransfer, YoBoolean isDoubleSupport, ReferenceFrame trajectoryFrame,
                                          YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.trajectoryFrame = trajectoryFrame;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isStanding = isStanding;
      this.isInitialTransfer = isInitialTransfer;
      this.isDoubleSupport = isDoubleSupport;

      areICPDynamicsSatisfied = new YoBoolean("areICPDynamicsSatisfied", registry);
      areICPDynamicsSatisfied.set(false);
      areCoMDynamicsSatisfied = new YoBoolean("areCoMDynamicsSatisfied", registry);
      areCoMDynamicsSatisfied.set(false);

      continuouslyAdjustForICPContinuity = new YoBoolean("continuouslyAdjustForICPContinuity", registry);
      continuouslyAdjustForICPContinuity.set(CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY);

      totalNumberOfCMPSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);

      startTimeOfCurrentPhase = new YoDouble(namePrefix + "StartTimeCurrentPhase", registry);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "LocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);

      yoICPVelocityDynamicsCurrent = new YoFrameVector("ICPVelocityDynamics", ReferenceFrame.getWorldFrame(), registry);

      cmpTrajectoryLength = new YoInteger("CMPTrajectoryLength", registry);
      icpTerminalLength = new YoInteger("ICPTerminalLength", registry);
      currentSegmentIndex = new YoInteger("CurrentSegment", registry);

      icpQuantityInitialConditionList.add(new FramePoint3D());
      while (icpDesiredInitialPositions.size() < defaultSize)
      {
         icpDesiredInitialPositions.add(new FramePoint3D());
         icpDesiredFinalPositions.add(new FramePoint3D());
         
         icpDesiredInitialPositionsFromCoPs.add(new FramePoint3D());
         icpDesiredFinalPositionsFromCoPs.add(new FramePoint3D());
         
         cmpDesiredFinalPositions.add(new FramePoint3D());

         comDesiredInitialPositions.add(new FramePoint3D());
         comDesiredFinalPositions.add(new FramePoint3D());

         icpQuantityInitialConditionList.add(new FrameVector3D());
      }
   }

   public void setNumberOfRegisteredSteps(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void reset()
   {
      cmpTrajectories.clear();
      totalNumberOfCMPSegments.set(0);
      localTimeInCurrentPhase.set(0.0);
   }
   
   public void resetCoPs()
   {
      copTrajectories.clear();
   }

   public void initializeForTransfer(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                     List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }

         SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }

      
      numberOfSegmentsTransfer0 = transferCMPTrajectories.get(0).getNumberOfSegments();
      
      initialize();
   }

   public void initializeForSwing(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                  List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(0);
      int cmpSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }

      numberOfSegmentsSwing0 = swingCMPTrajectories.get(0).getNumberOfSegments();
      
      initialize();
   }
   
   public void initializeForTransferFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                             List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();
      
      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         int copSegments = transferCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
         }
         
         SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         copSegments = swingCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(swingCoPTrajectory.getSegment(copSegment));
         }
      }
      
      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      int copSegments = transferCoPTrajectory.getNumberOfSegments();
      for (int copSegment = 0; copSegment < copSegments; copSegment++)
      {
         copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
      }
      
      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());
      
      if(isInitialTransfer.getBooleanValue())
      {
         setICPInitialConditionsForAdjustment(localTimeInCurrentPhase.getDoubleValue(), -1);            
      }
   }
   
   public void initializeForSwingFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                          List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();

      SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(0);
      int copSegments = swingCoPTrajectory.getNumberOfSegments();
      for (int copSegment = 0; copSegment < copSegments; copSegment++)
      {
         copTrajectories.add(swingCoPTrajectory.getSegment(copSegment));
      }

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         copSegments = transferCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
         }

         swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         copSegments = swingCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(swingCoPTrajectory.getSegment(copSegment));
         }
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      copSegments = transferCoPTrajectory.getNumberOfSegments();
      for (int copSegment = 0; copSegment < copSegments; copSegment++)
      {
         copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
      }
      
      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());
   }

   @Override
   public void initialize()
   {
      if (isInitialTransfer.getBooleanValue())
      {
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(0);
         cmpPolynomial3D.compute(cmpPolynomial3D.getInitialTime());
         comPositionDesiredInitialCurrentSegment.set(cmpPolynomial3D.getFramePosition());
      }
      else
      {
         comPositionDesiredInitialCurrentSegment.set(comPositionDesiredCurrent); // TODO; set to 1*dt after comPositionDesiredCurrent
      }
      // FIXME
      //      if (!isStanding.getBooleanValue())
      {

         icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());

         icpPositionDesiredTerminal.set(icpDesiredFinalPositions.get(cmpTrajectories.size() - 1));

         cmpTrajectoryLength.set(cmpTrajectories.size());
         icpTerminalLength.set(icpDesiredFinalPositions.size());
      }

   }
   
   public void setICPInitialConditionsForAdjustment(double time, int currentSwingSegment)
   {
         icpAdjustmentToolbox.setICPInitialConditions(time, icpDesiredFinalPositionsFromCoPs, copTrajectories, currentSwingSegment, isInitialTransfer.getBooleanValue(),
                                                      omega0.getDoubleValue());    
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing()
   {
      if ((isInitialTransfer.getBooleanValue() || (isDoubleSupport.getBooleanValue() && continuouslyAdjustForICPContinuity.getBooleanValue()))
            && copTrajectories.size() > 1)
      {
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories,
                                                                           omega0.getDoubleValue());
      }
      reset();
   }

   public void initializeCenterOfMass()
   {
      comToolbox.computeDesiredCenterOfMassCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, comDesiredInitialPositions,
                                                        comDesiredFinalPositions, cmpTrajectories, comPositionDesiredInitialCurrentSegment,
                                                        omega0.getDoubleValue());
   }

   @Override
   public void compute(double time)
   {
      if (!isStanding.getBooleanValue())
      {
         localTimeInCurrentPhase.set(time - startTimeOfCurrentPhase.getDoubleValue());

         currentSegmentIndex.set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), cmpTrajectories));
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(currentSegmentIndex.getIntegerValue());
         getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalCurrentSegment, currentSegmentIndex.getIntegerValue());
         getCoMPositionDesiredInitialFromSegment(comPositionDesiredInitialCurrentSegment, currentSegmentIndex.getIntegerValue());
         getCoMPositionDesiredFinalFromSegment(comPositionDesiredFinalCurrentSegment, currentSegmentIndex.getIntegerValue());

         // ICP
         icpToolbox.computeDesiredCapturePointPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                                icpPositionDesiredFinalCurrentSegment, cmpPolynomial3D, icpPositionDesiredCurrent);
         icpToolbox.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                                icpPositionDesiredFinalCurrentSegment, cmpPolynomial3D, icpVelocityDesiredCurrent);
         icpToolbox.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                                    icpPositionDesiredFinalCurrentSegment, cmpPolynomial3D, icpAccelerationDesiredCurrent);

         // CoM
         comToolbox.computeDesiredCenterOfMassPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                       icpPositionDesiredFinalCurrentSegment, comPositionDesiredInitialCurrentSegment, cmpPolynomial3D,
                                                       comPositionDesiredCurrent);
         comToolbox.computeDesiredCenterOfMassVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                       icpPositionDesiredFinalCurrentSegment, comPositionDesiredInitialCurrentSegment, cmpPolynomial3D,
                                                       comVelocityDesiredCurrent);
         comToolbox.computeDesiredCenterOfMassAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                           icpPositionDesiredFinalCurrentSegment, comPositionDesiredInitialCurrentSegment, cmpPolynomial3D,
                                                           comAccelerationDesiredCurrent);
         
         if(!isDoubleSupport.getBooleanValue())
         {
            setICPInitialConditionsForAdjustment(localTimeInCurrentPhase.getDoubleValue(), currentSegmentIndex.getIntegerValue()); // TODO: add controller dt for proper continuation
         }
         
         checkICPDynamics(localTimeInCurrentPhase.getDoubleValue(), icpVelocityDesiredCurrent, icpPositionDesiredCurrent, cmpPolynomial3D);
         checkCoMDynamics(localTimeInCurrentPhase.getDoubleValue(), comVelocityDesiredCurrent, icpPositionDesiredCurrent, comPositionDesiredCurrent);
      }
   }

   private void checkICPDynamics(double time, FrameVector3D icpVelocityDesiredCurrent, FramePoint3D icpPositionDesiredCurrent, FrameTrajectory3D cmpPolynomial3D)
   {
      cmpPolynomial3D.compute(time);

      icpVelocityDynamicsCurrent.sub(icpPositionDesiredCurrent, cmpPolynomial3D.getFramePosition());
      icpVelocityDynamicsCurrent.scale(omega0.getDoubleValue());
      
      yoICPVelocityDynamicsCurrent.set(icpVelocityDynamicsCurrent);

      areICPDynamicsSatisfied.set(icpVelocityDesiredCurrent.epsilonEquals(icpVelocityDynamicsCurrent, 10e-5));
   }

   private void checkCoMDynamics(double time, FrameVector3D comVelocityDesiredCurrent, FramePoint3D icpPositionDesiredCurrent, FramePoint3D comPositionDesiredCurrent)
   {
      comVelocityDynamicsCurrent.sub(icpPositionDesiredCurrent, comPositionDesiredCurrent);
      comVelocityDynamicsCurrent.scale(omega0.getDoubleValue());

      areCoMDynamicsSatisfied.set(comVelocityDesiredCurrent.epsilonEquals(comVelocityDynamicsCurrent, 10e-5));
   }

   private int getCurrentSegmentIndex(double timeInCurrentPhase, List<FrameTrajectory3D> cmpTrajectories)
   {
      int currentSegmentIndex = FIRST_SEGMENT;
      while (timeInCurrentPhase > cmpTrajectories.get(currentSegmentIndex).getFinalTime()
            && Math.abs(cmpTrajectories.get(currentSegmentIndex).getFinalTime() - cmpTrajectories.get(currentSegmentIndex + 1).getInitialTime()) < 1.0e-5)
      {
         currentSegmentIndex++;
         if (currentSegmentIndex + 1 > cmpTrajectories.size())
         {
            return currentSegmentIndex;
         }
      }
      return currentSegmentIndex;
   }

   public void getICPPositionDesiredFinalFromSegment(FramePoint3D icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment));
   }

   public void getCoMPositionDesiredInitialFromSegment(FramePoint3D comPositionDesiredInitial, int segment)
   {
      comPositionDesiredInitial.set(comDesiredInitialPositions.get(segment));
   }

   public void getCoMPositionDesiredFinalFromSegment(FramePoint3D comPositionDesiredFinal, int segment)
   {
      comPositionDesiredFinal.set(comDesiredFinalPositions.get(segment));
   }

   public void getCoMPosition(YoFramePoint comPositionToPack)
   {
      comPositionToPack.set(comPositionDesiredCurrent);
   }

   public void getCoMVelocity(YoFrameVector comVelocityToPack)
   {
      comVelocityToPack.set(comVelocityDesiredCurrent);
   }

   public void getCoMAcceleration(YoFrameVector comAccelerationToPack)
   {
      comAccelerationToPack.set(comAccelerationDesiredCurrent);
   }

   public void getFinalCoMPositionInSwing(FramePoint3D comPositionDesiredFinalToPack)
   {
      if(isDoubleSupport.getBooleanValue())
      {
         getCoMPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsTransfer0 + numberOfSegmentsSwing0 - 1);
      }
      else
      {
         getCoMPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsSwing0 - 1);
      }
   }
   
   public void getFinalCoMPositionInTransfer(FramePoint3D comPositionDesiredFinalToPack)
   {
      if(isDoubleSupport.getBooleanValue())
      {
         getCoMPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsTransfer0 - 1);
      }
      else
      {
         getCoMPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsSwing0 + numberOfSegmentsTransfer0 - 1);
      }
   }


   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredCurrent);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
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

   public List<FramePoint3D> getICPPositionDesiredInitialList()
   {
      return icpDesiredInitialPositions;
   }

   public List<FramePoint3D> getICPPositionDesiredFinalList()
   {
      return icpDesiredFinalPositions;
   }

   public List<FramePoint3D> getCoMPositionDesiredInitialList()
   {
      return comDesiredInitialPositions;
   }

   public List<FramePoint3D> getCoMPositionDesiredFinalList()
   {
      return comDesiredFinalPositions;
   }

   public List<FramePoint3D> getCMPPositionDesiredList()
   {
      for (int i = 0; i < cmpTrajectories.size(); i++)
      {
         Trajectory cmpPolynomialX = cmpTrajectories.get(i).getTrajectory(0);
         Trajectory cmpPolynomialY = cmpTrajectories.get(i).getTrajectory(1);
         Trajectory cmpPolynomialZ = cmpTrajectories.get(i).getTrajectory(2);

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
      return totalNumberOfCMPSegments.getIntegerValue();
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
