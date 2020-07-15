package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedContactSequenceUpdater;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
//us.ihmc.euclid.tools.EuclidCoreIOTools.TimeInterval
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the simple Version of the BipedSimpleCoMTrajectoryPlanner: TODO: Update following explanation
 * Change from taking in {@link BipedTimedStep} to taking in {@link footstep} and {@link footsteptiming}
 * 
 * a base class for bipeds for dynamic trajectory planning. It is used to generate feasible DCM, CoM, and VRP trajectories. The inputs to this class
 * are a list of {@link BipedTimedStep}, which are converted to a list of {@link ContactStateProvider}, which is then used by the {@link CoMTrajectoryPlanner}.
 * This is done using {@link BipedContactSequenceUpdater} class.
 *
 * <p>
 * WARNING: This class current generates garbage from the {@link BipedContactSequenceTools#computeStepTransitionsFromStepSequence}.
 * </p>
 */

/*
 * 1. instantiated with a sideDependentList of SoleFrames (  ), gravity, nominal height, omega0, parent Registry, and yographics registry
 * 2. Add footsteps (footstep and footsteptiming) to the step sequence
 * 3. List of contact state providers created for the SimpleBipedCoMTrajectoryPlanner
 * 4. Solve for trajectory in the planner
 * 5. Compute the planner to get the desired vrp and such
 */
public class SimpleBipedCoMTrajectoryPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final String namePrefix = "";
   
   private final BipedContactSequenceUpdater sequenceUpdater;
   private final SimpleCoMTrajectoryPlanner comTrajectoryPlanner;

   private final YoDouble timeInContactPhase = new YoDouble(namePrefix + "timeInContactPhase", registry);

   private final List<ContactStateProvider> InputstepSequence = new ArrayList<>();
   private final List<ContactStateProvider> ControlStepSequence = new ArrayList<>();
   private final List<BipedTimedStep> stepSequence = new ArrayList<>();
   
   private final List<Footstep> footstepList = new ArrayList<>();
   private final List<FootstepTiming> footstepTimingList = new ArrayList<>();
   private final List<RobotSide> feetInContact = new ArrayList<>();
   
   private final boolean UseConvertedCSP = true;
   
   protected final YoEnum<RobotSide> transferToSide = new YoEnum<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> previousTransferToSide = new YoEnum<>(namePrefix + "PreviousTransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> supportSide = new YoEnum<>(namePrefix + "SupportSide", registry, RobotSide.class, true);
   
   //protected final YoInteger numberFootstepsToConsider = new YoInteger(namePrefix + "NumberFootstepsToConsider", registry);
   
   //Sole frames pass in the reference frames attached to the center of the bottom of the foot. Can be used to get the current 
   public SimpleBipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleFrames, double gravityZ, double nominalCoMHeight,
                                          DoubleProvider omega0, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      sequenceUpdater = new BipedContactSequenceUpdater(soleFrames, registry, yoGraphicsListRegistry);
      comTrajectoryPlanner = new SimpleCoMTrajectoryPlanner(omega0);
      comTrajectoryPlanner.setNominalCoMHeight(nominalCoMHeight);
      ((SimpleCoMTrajectoryPlanner) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

      parentRegistry.addChild(registry);
   }
   
   
   
   public void clearInputStepSequence()
   {
      InputstepSequence.clear();
   }
   
   public void clearConvertedStepSequence()
   {
      stepSequence.clear();
      footstepList.clear();
      footstepTimingList.clear();
   }
   
   // TODO: correct footstep inputs
   public void addStepToSequence(Footstep footstep, FootstepTiming timing)
   {
      /*
       * the list of footsteps are the timings and final positions of the footsteps. 
       * To create the contact states the first state must go from zero position to the first footstep, 
       * then the last state goes from the last footstep to the end position.
       */
      /*
       * BipedTimed Steps use the same information as footsteps and footsteptimings. There is already a tool to 
       * convert a list of Biped Timed steps to ContactStateProviders. THe  BipedContactSequenceUpdater
       * THis uses the soleframes input in the planner to get current position when planning for the beginning of the plan
       */
      
      addStepToSequence(footstep, timing, new FootstepShiftFractions(), 0);
      
   }
   
   
   public void addStepToSequence(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions, double CurrentTime)
   {
      footstepList.add(footstep);
      footstepTimingList.add(timing);
   }
   
   
   public void addStepToSequence(ContactStateProvider step)
   {
      InputstepSequence.add(step);
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }
   
   public void initialize()
   {
      sequenceUpdater.initialize();
   }
   
   //TODO: what list is input? - look at previous use of bipedCOMTrajectoryPlanner
   //feetInContact is a list of RobotSides that lists all the feet in contact for the current state
   public void solveForTrajectory(double currentTime,FramePoint3DReadOnly centerOfMassPosition)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, new FrameVector3D());
      if (UseConvertedCSP)
      {
         convertFootstepToBTS(currentTime);
      }
      else
      {
         ControlStepSequence.clear();
         ControlStepSequence.addAll(InputstepSequence);
         comTrajectoryPlanner.solveForTrajectory(ControlStepSequence);
      }
      compute(currentTime, centerOfMassPosition);
   }
   
   public void convertFootstepToBTS(double currentTime)
   {
      stepSequence.clear();
      double StepStartTime = currentTime;

      for(int i = 0; i<footstepList.size(); i++) 
      {
         BipedTimedStep step = new BipedTimedStep();
         step.getTimeInterval().setInterval(StepStartTime, StepStartTime + footstepTimingList.get(i).getSwingTime());
         step.setRobotSide(footstepList.get(i).getRobotSide());
         step.setGoalPose(footstepList.get(i).getFootstepPose());
         stepSequence.add(step);
  
         StepStartTime += footstepTimingList.get(i).getStepTime();
      }  
   }
   
   public void compute(double currentTime, FramePoint3DReadOnly centerOfMassPosition)
   {
      if (UseConvertedCSP)
      {
         feetInContact.clear();
         for (RobotSide robotSide : RobotSide.values)
            feetInContact.add(robotSide);
         //feetInContact.add(RobotSide.RIGHT);
         sequenceUpdater.update(stepSequence, feetInContact, currentTime);
         ControlStepSequence.addAll(sequenceUpdater.getContactSequence());
         comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, new FrameVector3D());
         comTrajectoryPlanner.solveForTrajectory(ControlStepSequence);
      }
      int Curri = getSegmentNumber(currentTime);
      double timeInPhase = currentTime - ControlStepSequence.get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);
      comTrajectoryPlanner.compute(Curri, timeInContactPhase.getDoubleValue());
   }
   
   
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return comTrajectoryPlanner.getDesiredDCMPosition();
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return comTrajectoryPlanner.getDesiredDCMVelocity();
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return comTrajectoryPlanner.getDesiredCoMPosition();
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return comTrajectoryPlanner.getDesiredCoMVelocity();
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return comTrajectoryPlanner.getDesiredCoMAcceleration();
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }
   
   public List<Trajectory3D> getVRPTrajectories()
   {
      return comTrajectoryPlanner.getVRPTrajectories();
   }
   
   public int getSegmentNumber(double currentTime)
   {
      for (int i = 0; i < ControlStepSequence.size(); i++)
      {
         if (ControlStepSequence.get(i).getTimeInterval().intervalContains(currentTime))
            return i;
      }

      return ControlStepSequence.size() - 1;
   }
   
   public double getTimeInPhase(double currentTime, int phase)
   {
      return currentTime - ControlStepSequence.get(phase).getTimeInterval().getStartTime();
   }
   
   /*
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }
   
   public void setTransferToSide(RobotSide robotSide)
   {
      previousTransferToSide.set(transferToSide.getEnumValue());
      transferToSide.set(robotSide);
   }
   
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
      {
         previousTransferToSide.set(transferToSide.getEnumValue());
         transferToSide.set(robotSide.getOppositeSide());
      }
      else
      {
         transferToSide.set(null);
      }
   }
   
   public void endTick()
   {
      
   }
   
   public double estimateTimeRemainingForStateUnderDisturbance(FramePoint2D actualCapturePointPosition)
   {
      if (isDone())
         return 0.0;

      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(actualCapturePointPosition);

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      double estimatedTimeRemaining = getTimeInCurrentStateRemaining() - deltaTimeToBeAccounted;
      estimatedTimeRemaining = MathTools.clamp(estimatedTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      return estimatedTimeRemaining;
   }
   
   private final FramePoint2D desiredICP2d = new FramePoint2D();
   private final FramePoint2D finalICP2d = new FramePoint2D();
   private final FrameLine2D desiredICPToFinalICPLine = new FrameLine2D();
   private final FrameLineSegment2D desiredICPToFinalICPLineSegment = new FrameLineSegment2D();
   private final FramePoint2D actualICP2d = new FramePoint2D();
   
   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2DReadOnly actualCapturePointPosition)
   {
      desiredICP2d.setIncludingFrame(getDesiredDCMPosition());
      finalICP2d.setIncludingFrame(stepSequence.);

      if (desiredICP2d.distance(finalICP2d) < 1.0e-10)
         return Double.NaN;

      desiredICPToFinalICPLineSegment.set(desiredICP2d, finalICP2d);
      actualICP2d.setIncludingFrame(actualCapturePointPosition);
      double percentAlongLineSegmentICP = desiredICPToFinalICPLineSegment.percentageAlongLineSegment(actualICP2d);
      if (percentAlongLineSegmentICP < 0.0)
      {
         desiredICPToFinalICPLine.set(desiredICP2d, finalICP2d);
         desiredICPToFinalICPLine.orthogonalProjection(actualICP2d);
      }
      else
      {
         desiredICPToFinalICPLineSegment.orthogonalProjection(actualICP2d);
      }

      double actualDistanceDueToDisturbance = desiredCMPPosition.distanceXY(actualICP2d);
      double expectedDistanceAccordingToPlan = desiredCMPPosition.distanceXY(desiredICPPosition);

      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega0.getDoubleValue();
   
      return 0;
   }
   
   public double getTimeInCurrentStateRemaining()
   {
      return 0;
   }
   
   public void initializeForStanding(double time)
   {
      
   }
   
   public void initializeForSingleSupport(double time)
   {
      
   }
   
   public void initializeForTransfer(double time)
   {
      
   }
   
   public void holdCurrentICP(FramePoint3D tempCapturePoint)
   {
      
   }
   
   public void getFinalDesiredCapturePointPosition(YoFramePoint2D yoFinalDesiredICP)
   {
      
   }
   
   public void getFinalDesiredCenterOfMassPosition(YoFramePoint3D yoFinalDesiredCoM)
   {
      
   }
   
   public void setFinalTransferWeightDistribution(double weightDistribution)
   {
      
   }
   
   public void setFinalTransferDurationAlpha(double finalTransferSplitFraction)
   {
      
   }
   
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      
   }
   
   public void updateCurrentPlan()
   {
      
   }
   
   public boolean isDone()
   {
      return true;
   }
   */
   
}
