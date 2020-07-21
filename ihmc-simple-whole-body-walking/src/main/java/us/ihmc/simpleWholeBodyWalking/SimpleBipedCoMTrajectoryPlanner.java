package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
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
import us.ihmc.yoVariables.variable.YoBoolean;
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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
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

public class SimpleBipedCoMTrajectoryPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimpleBipedContactSequenceUpdater sequenceUpdater;
   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;
   
   private static final double ZERO_TIME = 0.0;
   public static final double SUFFICIENTLY_LARGE = 100.0;
   
   protected final YoEnum<RobotSide> transferToSide = new YoEnum<>("TransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> previousTransferToSide = new YoEnum<>("PreviousTransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> supportSide = new YoEnum<>("SupportSide", registry, RobotSide.class, true);
   
   
   private final YoInteger numberOfUpcomingFootsteps = new YoInteger("NumberOfUpcomingFootsteps", registry);;
   private final List<Footstep> upcomingFootsteps = new ArrayList<>();
   private final List<FootstepTiming> upcomingFootstepTimings = new ArrayList<>();
   

   //TODO: CurrentState Management
   protected final YoInteger numberFootstepsToConsider = new YoInteger("NumberFootstepsToConsider", registry);
   protected final YoBoolean isStanding = new YoBoolean("IsStanding", registry);
   protected final YoBoolean isInitialTransfer = new YoBoolean("IsInitialTransfer", registry);
   protected final YoBoolean isFinalTransfer = new YoBoolean("IsFinalTransfer", registry);
   protected final YoBoolean isDoubleSupport = new YoBoolean("IsDoubleSupport", registry);
   /** Time at which the current state was initialized. */
   protected final YoDouble initialTime = new YoDouble("CurrentStateInitialTime", registry);
   /** Time spent in the current state. */
   protected final YoDouble timeInCurrentState = new YoDouble("TimeInCurrentState", registry);
   protected final YoDouble clampedTimeInCurrentState = new YoDouble("ClampedTimeInCurrentState", registry);
   /** Time remaining before the end of the current state. */
   protected final YoDouble timeInCurrentStateRemaining = new YoDouble("RemainingTime", registry);
   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);
   private Footstep currentFootstep;
   private FootstepTiming currentFootstepTiming;
   
   //TODO: Velocity decay stuff
   
   //TODO: Holding Position
   protected final YoBoolean isHoldingPosition = new YoBoolean("IsHoldingPosition", registry);
   protected final YoFramePoint3D icpPositionToHold = new YoFramePoint3D("CapturePointPositionToHold", worldFrame, registry);

   //TODO: Angular Dynamics
   /** Desired Centroidal Angular Momentum (CAM) */
   final YoFrameVector3D desiredCentroidalAngularMomentum = new YoFrameVector3D("DesiredCentroidalAngularMomentum", worldFrame, registry);
   /** Desired Centroidal Torque (CT) */
   final YoFrameVector3D desiredCentroidalTorque = new YoFrameVector3D("DesiredCentroidalTorque", worldFrame, registry);
   
   //TODO: Adjusting Plan for Continuity?
   
   //TODO: Currently doesn't use Gravity?
   
   //TODo: Visualizations
   private static final boolean VISUALIZE = true;
   
   //TODO: Set Up YoTime Here
   
   //TODO: InitializeParameters with icpPlannerParameters with default Parameters
   //alpha*SwingDuration is % of swingDuration spend with DCM before exit Corner point
   protected final YoDouble defaultSwingDurationAlpha = new YoDouble("DefaultSwingDurationAlpha",
                                                                     "Repartition of the swing duration around the exit corner point.", registry);
   private final YoDouble defaultSwingDurationShiftFraction = new YoDouble("DefaultSwingDurationShiftFraction", registry);
   //
   protected final YoDouble defaultTransferDurationAlpha = new YoDouble("DefaultTransferDurationAlpha",
                                                                        "Repartition of the transfer duration around the entry corner point.", registry);
   private final YoDouble finalTransferDurationAlpha = new YoDouble("FinalTransferDurationAlpha", registry);
   //
   private final YoDouble defaultTransferWeightDistribution = new YoDouble("DefaultWeightDistribution", registry);
   private final YoDouble finalTransferWeightDistribution = new YoDouble("FinalTransferWeightDistribution", registry);
   protected final YoDouble defaultFinalTransferDuration = new YoDouble("DefaultFinalTransferDuration", registry);
   protected final YoDouble FinalTransferDuration = new YoDouble("FinalTransferDuration", registry);
   
   
   
   public SimpleBipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleZUpFrames, double gravityZ, double nominalCoMHeight,
                                          DoubleProvider omega0, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(soleZUpFrames, gravityZ, nominalCoMHeight, omega0, parentRegistry, yoGraphicsListRegistry,
           null, null, null);
   }
   
      public SimpleBipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleZUpFrames, double gravityZ, double nominalCoMHeight,
                                             DoubleProvider omega0, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                             YoDouble yoTime, ICPPlannerParameters icpPlannerParameters, BipedSupportPolygons bipedSupportPolygons)
      {
      sequenceUpdater = new SimpleBipedContactSequenceUpdater(soleZUpFrames, registry, yoGraphicsListRegistry);
      comTrajectoryPlanner = new SimpleCoMTrajectoryPlanner(omega0);
      comTrajectoryPlanner.setNominalCoMHeight(nominalCoMHeight);
      if (yoGraphicsListRegistry != null)
         ((SimpleCoMTrajectoryPlanner) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

      if (icpPlannerParameters != null)
         initializeParameters(icpPlannerParameters);
      parentRegistry.addChild(registry);
   }
   
   private void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      defaultTransferDurationAlpha.set(icpPlannerParameters.getTransferSplitFraction());
      defaultSwingDurationAlpha.set(icpPlannerParameters.getSwingSplitFraction());
      finalTransferDurationAlpha.set(icpPlannerParameters.getTransferSplitFraction());

      //velocityDecayDurationWhenDone.set(icpPlannerParameters.getVelocityDecayDurationWhenDone());
      //velocityReductionFactor.set(Double.NaN);

      numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());

      //referenceCoPGenerator.initializeParameters(icpPlannerParameters);
      //referenceCMPGenerator.setGroundReaction(robotMass * gravityZ);
      //angularMomentumTrajectoryGenerator.initializeParameters(icpPlannerParameters, robotMass, gravityZ);
      defaultSwingDurationShiftFraction.set(icpPlannerParameters.getSwingDurationShiftFraction());

      //adjustPlanForSSContinuity.set(icpPlannerParameters.adjustCoPPlanForSingleSupportContinuity());
      //adjustPlanForDSContinuity.set(icpPlannerParameters.adjustEveryCoPPlanForDoubleSupportContinuity());
      //adjustPlanForInitialDSContinuity.set(icpPlannerParameters.adjustInitialCoPPlanForDoubleSupportContinuity());
      //adjustPlanForStandingContinuity.set(icpPlannerParameters.adjustCoPPlanForStandingContinuity());

      //doContinuousReplanningForStanding.set(icpPlannerParameters.doContinuousReplanningForStanding());
      //doContinuousReplanningForTransfer.set(icpPlannerParameters.doContinuousReplanningForTransfer());
      //doContinuousReplanningForSwing.set(icpPlannerParameters.doContinuousReplanningForSwing());
   }
   
   //TODO: Catch bad steps
   public void addStepToSequence(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions)
   {
      upcomingFootsteps.add(footstep);
      upcomingFootstepTimings.add(timing);
   }
   
   public void clearPlan()
   {
      upcomingFootsteps.clear();
      upcomingFootstepTimings.clear();
   }
   public void clearStepSequence()
   {
      upcomingFootsteps.clear();
      upcomingFootstepTimings.clear();
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void initialize()
   {
      sequenceUpdater.initialize();
   }

   
   public double computeSetpoints(double currentTime)
   {
      return computeSetpoints(currentTime, getCurrentFeetInContact());
   }
   
   public double computeSetpoints(double currentTime, List<RobotSide> currentFeetInContact)
   {
      trimPastSteps(currentTime);
      //Update newest state - note current state refers to swing or trasnfer within the step start and End transfer per step?
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue());
      timeInCurrentStateRemaining.set(getCurrentStateDuration() - timeInCurrentState.getDoubleValue());
         
      sequenceUpdater.update(upcomingFootsteps, upcomingFootstepTimings, currentFeetInContact, currentTime);

      double timeInPhase = currentTime - sequenceUpdater.getAbsoluteContactSequence().get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      comTrajectoryPlanner.solveForTrajectory(sequenceUpdater.getContactSequence());
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());
      
      return timeInPhase;
   }

   private void trimPastSteps(double currentTime)
   {
      int i = 0;
      while (i<upcomingFootsteps.size())
      {
         double stepEndTime = upcomingFootstepTimings.get(i).getExecutionStartTime() + upcomingFootstepTimings.get(i).getSwingStartTime()
               + upcomingFootstepTimings.get(i).getStepTime();
         if (stepEndTime < currentTime)
         {
            upcomingFootsteps.remove(i);
            upcomingFootstepTimings.remove(i);
         }
         else
         {
            i++;
         }
      }
   }

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
      if (robotSide == null)
      {
         transferToSide.set(null);
      }
      else
      {
         previousTransferToSide.set(transferToSide.getEnumValue());
         transferToSide.set(robotSide.getOppositeSide());
      }
   }
   
   private List<RobotSide> getCurrentFeetInContact()
   {
      List<RobotSide> currentFeetInContact = new ArrayList<>();
      if (isDoubleSupport.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
            currentFeetInContact.add(robotSide);
      }
      else
      {
         currentFeetInContact.add(supportSide.getEnumValue());         
      }
 
      return currentFeetInContact;
   }
   
   public void setDefaultPhaseTimes(double defaultSwingTime, double defaultTransferTime)
   {
      // TODO Auto-generated method stub
      
   }
   
   public void updateCurrentPlan()
   {

   }
   
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   { //This state doesn't seem to be really used
      this.icpPositionToHold.set(icpPositionToHold);
      isInitialTransfer.set(false);
      isFinalTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(true);
      isHoldingPosition.set(true);
      // Asking the CoP and ICP to be the same since we assume that holds can be requested in static conditions only
   }

   public void initializeForStanding(double currentTime)
   {
      clearPlan();

      previousTransferToSide.set(null);

      this.initialTime.set(currentTime);
      isInitialTransfer.set(true);
      isFinalTransfer.set(false);
      isStanding.set(true);
      isDoubleSupport.set(true);
      isHoldingPosition.set(false);
      FinalTransferDuration.set(defaultFinalTransferDuration.getDoubleValue());

      updateAbsoluteTimings(currentTime);
   }

   public void initializeForSingleSupport(double currentTime)
   {
      this.initialTime.set(currentTime);
      isInitialTransfer.set(false);
      isFinalTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(false);
      isHoldingPosition.set(false);
      
      FinalTransferDuration.set(upcomingFootstepTimings.get(0).getTransferTime());

      updateAbsoluteTimings(currentTime);
   }

   public void initializeForTransfer(double currentTime)
   {
      this.initialTime.set(currentTime);
      //coming from standing or just continuing initial Transfer 
      isInitialTransfer.set(isStanding.getBooleanValue() || isInitialTransfer.getBooleanValue());
      isStanding.set(false);
      isDoubleSupport.set(true);
      isHoldingPosition.set(false);

      if (isInitialTransfer.getBooleanValue() || isFinalTransfer.getValue())
         previousTransferToSide.set(null);
      
      //int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      //isFinalTransfer.set(numberOfFootstepRegistered == 0);
      
      updateAbsoluteTimings(currentTime);
   }
   
   /*
    * Takes initial time of state and uses the swing and transfer times of the FootstepTimings to add absolute times to the footsteps
    */
   public void updateAbsoluteTimings(double initialTime)
   {
      if (upcomingFootstepTimings.size() == 0)
         return;
      
      //if initial transfer havent entered footsteps yet
      double trackingTime = initialTime;
      if (isInitialTransfer.getBooleanValue())
         trackingTime += defaultFinalTransferDuration.getDoubleValue();
      else if (isDoubleSupport.getBooleanValue())
         trackingTime += FinalTransferDuration.getDoubleValue();
      
      for (int i = 0; i < upcomingFootstepTimings.size(); i++)
      {
         upcomingFootstepTimings.get(i).setAbsoluteTime(0.01, trackingTime);
         trackingTime += upcomingFootstepTimings.get(i).getStepTime();
      }
   }

   public void setFinalTransferWeightDistribution(double weightDistribution)
   {
      // TODO Auto-generated method stub
      
   }

   public void setFinalTransferDurationAlpha(double finalTransferSplitFraction)
   {
      // TODO Auto-generated method stub
      
   }

   public void setFinalTransferDuration(double duration)
   {
      if (duration < 0.01)
         return;

      defaultFinalTransferDuration.set(duration);      
   }
   
   public void setNumberOfFootstepsToConsider(int numberStepsToConsider)
   {
      numberFootstepsToConsider.set(numberStepsToConsider);
   }
   
   public int getNumberOfFootstepsToConsider()
   {
      return numberFootstepsToConsider.getIntegerValue();
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
   
   public FramePoint3DReadOnly getDesiredCOPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }

   public List<Trajectory3D> getVRPTrajectories()
   {
      return ((SimpleCoMTrajectoryPlanner) comTrajectoryPlanner).getVRPTrajectories();
   }
   
   public List<Footstep> getUpcomingFootsteps()
   {
      return upcomingFootsteps;
   }
   
   public List<FootstepTiming> getUpcomingFootstepTimings()
   {
      return upcomingFootstepTimings;
   }

   
   
   public void getFinalDesiredCapturePointPosition(YoFramePoint2D yoFinalDesiredICP)
   {
      // TODO Auto-generated method stub
      
   }
   
   public void getFinalDesiredCenterOfMassPosition(YoFramePoint3D yoFinalDesiredCoM)
   {
      // TODO Auto-generated method stub
      
   }

   public void getNextExitCMP(FramePoint3D entryCMPToPack)
   {
      // TODO Auto-generated method stub
      
   }

   public double getCurrentStateDuration()
   {
      if (isDoubleSupport.getBooleanValue())
         return getTransferDuration(0);
      else
         return getSwingDuration(0);
   }
   
   public double getTransferDuration(int stepNumber)
   {
      if (isStanding.getBooleanValue() || upcomingFootsteps.size() == 0)
         return SUFFICIENTLY_LARGE;
      else if (isInitialTransfer.getBooleanValue() || isFinalTransfer.getBooleanValue())
         return defaultFinalTransferDuration.getDoubleValue();
      else
         return upcomingFootstepTimings.get(stepNumber).getTransferTime();
   }

   public double getSwingDuration(int stepNumber)
   {
      return upcomingFootstepTimings.get(stepNumber).getSwingTime();
   }
   
   public double getTimeInContactPhase()
   {
      return timeInContactPhase.getDoubleValue();
   }
   
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   public double estimateTimeRemainingForStateUnderDisturbance(FramePoint2D capturePoint2d)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   public boolean isDone()
   {
      if (timeInCurrentStateRemaining.isNaN())
      {
         return true;
      }
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.0;//02; //Don't fudge it here 
      //fudge that the footstep is actually planned to be 0.01 longer than the state machine thinks
   }

   public void endTick()
   {
      // TODO Auto-generated method stub
      
   }
}

