package us.ihmc.simpleWholeBodyWalking;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedContactSequenceTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedContactSequenceUpdater;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

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
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimpleBipedContactSequenceUpdater sequenceUpdater;
   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;
   
   public static final double SUFFICIENTLY_LARGE = 100.0;
   
   protected final YoEnum<RobotSide> transferToSide = new YoEnum<>("TransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> previousTransferToSide = new YoEnum<>("PreviousTransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> supportSide = new YoEnum<>("SupportSide", registry, RobotSide.class, true);
   
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
   
   protected final YoBoolean isHoldingPosition = new YoBoolean("IsHoldingPosition", registry);
   protected final YoFramePoint3D icpPositionToHold = new YoFramePoint3D("CapturePointPositionToHold", worldFrame, registry);
   
   protected final YoDouble defaultFinalTransferDuration = new YoDouble("DefaultFinalTransferDuration", registry);
   protected final YoDouble finalTransferDuration = new YoDouble("FinalTransferDuration", registry);
   
   private static final boolean VISUALIZE = true;
   
   public SimpleBipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleZUpFrames, double gravityZ, double nominalCoMHeight,
                                          DoubleProvider omega0, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(soleZUpFrames, gravityZ, nominalCoMHeight, omega0, parentRegistry, yoGraphicsListRegistry,
           null, null);
   }
   
      public SimpleBipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleZUpFrames, double gravityZ, double nominalCoMHeight,
                                             DoubleProvider omega0, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                             YoDouble yoTime, BipedSupportPolygons bipedSupportPolygons)
      {
      sequenceUpdater = new SimpleBipedContactSequenceUpdater(soleZUpFrames, registry, yoGraphicsListRegistry);
      comTrajectoryPlanner = new SimpleCoMTrajectoryPlanner(omega0);
      comTrajectoryPlanner.setNominalCoMHeight(nominalCoMHeight);
      
      if (yoGraphicsListRegistry != null && VISUALIZE)
         ((SimpleCoMTrajectoryPlanner) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

         numberFootstepsToConsider.set(3);
      parentRegistry.addChild(registry);
      
      defaultFinalTransferDuration.set(0.5);
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void initialize()
   {
      sequenceUpdater.initialize();
   }
   
   public double computeSetpoints(double currentTime, List<Footstep> upcomingFootsteps, List<FootstepTiming> upcomingFootstepTimings)
   {

      double currentStateDuration;
      
      if (isDoubleSupport.getBooleanValue())
      {
         currentStateDuration = finalTransferDuration.getDoubleValue();
         sequenceUpdater.updateForDoubleSupport(upcomingFootsteps, upcomingFootstepTimings, previousTransferToSide.getEnumValue(), 
                                                initialTime.getDoubleValue(), currentStateDuration, currentTime, numberFootstepsToConsider.getIntegerValue());
         currentStateDuration = sequenceUpdater.getAbsoluteContactSequence().get(0).getTimeInterval().getDuration();
      }
      else
      {
         sequenceUpdater.updateForSingleSupport(upcomingFootsteps, upcomingFootstepTimings, supportSide.getEnumValue(), 
                                                initialTime.getDoubleValue(), currentTime, numberFootstepsToConsider.getIntegerValue());
         finalTransferDuration.set(upcomingFootstepTimings.get(0).getTransferTime());
         currentStateDuration = upcomingFootstepTimings.get(0).getSwingTime();
      }
      
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue());
      timeInCurrentStateRemaining.set(currentStateDuration - timeInCurrentState.getDoubleValue());
      
      double timeInPhase = currentTime - sequenceUpdater.getAbsoluteContactSequence().get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      comTrajectoryPlanner.solveForTrajectory(sequenceUpdater.getContactSequence());
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());
      
      return timeInPhase;
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
   
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   { //This state doesn't seem to be really used
      this.icpPositionToHold.set(icpPositionToHold);
      isInitialTransfer.set(false);
      isFinalTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(true);
      isHoldingPosition.set(true);
   }

   public void initializeForStanding(double currentTime)
   {
      previousTransferToSide.set(null);

      this.initialTime.set(currentTime);
      isInitialTransfer.set(true);
      isFinalTransfer.set(false);
      isStanding.set(true);
      isDoubleSupport.set(true);
      isHoldingPosition.set(false);
      finalTransferDuration.set(defaultFinalTransferDuration.getDoubleValue());
      //previousTransferToSide.set(null);
   }

   public void initializeForSingleSupport(double currentTime)
   {
      this.initialTime.set(currentTime);
      isInitialTransfer.set(false);
      isFinalTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(false);
      isHoldingPosition.set(false);
   }

   public void initializeForTransfer(double currentTime)
   {
      this.initialTime.set(currentTime);
      //coming from standing or just continuing initial Transfer 
      isInitialTransfer.set(isStanding.getBooleanValue() || isInitialTransfer.getBooleanValue());
      isStanding.set(false);
      isDoubleSupport.set(true);
      isHoldingPosition.set(false);
      if (isInitialTransfer.getBooleanValue())
         finalTransferDuration.set(defaultFinalTransferDuration.getDoubleValue());
      
      if(isInitialTransfer.getBooleanValue() || isFinalTransfer.getBooleanValue())
         previousTransferToSide.set(null);
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

   public List<? extends ContactStateProvider> getContactStateProviders()
   {
      return sequenceUpdater.getContactSequence();
   }
   
   public double getTimeInContactPhase()
   {
      return timeInContactPhase.getDoubleValue();
   }
   
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   public boolean isDone()
   {
      if (timeInCurrentStateRemaining.isNaN())
      {
         return true;
      }
     //Give a buffer of .01 s to ensure the controller ends the state within one timestep of the end time 
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.01;
   }

}

