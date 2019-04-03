package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleAdjustableFootstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Command that holds input for the {@link LinearMomentumRateControlModule} coming from the walking controller state
 * machine that might be running at a slower rate then the ICP feedback.
 *
 * @author Georg Wiedebach
 */
public class LinearMomentumRateControlModuleInput
{
   /**
    * The time constant for the LIPM model. This might be changed during a run so it is a parameter. Typically the value
    * is {@code omega0 = sqrt(gravity / comZ)}.
    */
   private double omega0;

   /**
    * The desired capture point that the ICP controller should track.
    */
   private final FramePoint2D desiredCapturePoint = new FramePoint2D();

   /**
    * The desired capture point velocity that the ICP controller should track.
    */
   private final FrameVector2D desiredCapturePointVelocity = new FrameVector2D();

   /**
    * Assuming to tracking error for the ICP this is the location that the CMP should be placed at according to the ICP
    * plan.
    */
   private final FramePoint2D perfectCMP = new FramePoint2D();

   /**
    * Assuming to tracking error for the ICP this is the location that the CoP should be placed at according to the ICP
    * plan.
    */
   private final FramePoint2D perfectCoP = new FramePoint2D();

   /**
    * Is a flag that enables the z-selection in the linear momentum rate command if {@code true}.
    */
   private boolean controlHeightWithMomentum;

   @Deprecated // The CoM height control should be moved to the fast thread or this should use the achieved value from the last tick.
   private double desiredCoMHeightAcceleration = 0.0;

   /**
    * Indicates which foot will be in support when stepping. Note, that this will only be used if
    * {@link #initializeForSingleSupport} is set to {@code true}.
    */
   private RobotSide supportSide = null;

   /**
    * Indicates which foot the robot shifts its weight to when performing a transfer. This will usually be the upcoming
    * support side. Note, that this will only be used if {@link #initializeForSingleSupport} is set to {@code true}.
    */
   private RobotSide transferToSide = null;

   /**
    * Flag that indicates the ICP planner has just transitioned to a standing state. This causes the ICP controller to
    * to initialize itself for standing.
    * <p>
    * Note, that this should only be true for one tick. Also, only one of the flags {@link #initializeForStanding},
    * {@link #initializeForSingleSupport}, and {@link #initializeForTransfer} can be true at a time.
    */
   private boolean initializeForStanding;

   /**
    * Flag that indicates the ICP planner has just transitioned to a single support state. This causes the ICP
    * controller to to initialize itself for single support.
    * <p>
    * Note, that this should only be true for one tick. Also, only one of the flags {@link #initializeForStanding},
    * {@link #initializeForSingleSupport}, and {@link #initializeForTransfer} can be true at a time.
    */
   private boolean initializeForSingleSupport;

   /**
    * Flag that indicates the ICP planner has just transitioned to a transfer state. This causes the ICP controller to
    * to initialize itself for transfer.
    * <p>
    * Note, that this should only be true for one tick. Also, only one of the flags {@link #initializeForStanding},
    * {@link #initializeForSingleSupport}, and {@link #initializeForTransfer} can be true at a time.
    */
   private boolean initializeForTransfer;

   /**
    * Flag that indicates to the ICP controller that the desired feedback CoP should stay within the bounds of the
    * support polygon. This should generally be true but can be set to false in certain cases where the support polygon
    * might not be accurate e.g. when using handholds.
    */
   private boolean keepCoPInsideSupportPolygon;

   /**
    * Is a flag that enables the z-selection in the angular momentum rate command if {@code true}. The desired angular
    * momentum will generally be zero.
    */
   private boolean minimizeAngularMomentumRateZ;

   /**
    * List of upcoming footsteps that are being executed by the controller. This is of interest to the ICP controller
    * because it might consider n-step capturability or adjust the locations of the upcoming footsteps when needed.
    * <p>
    * The fields {@link #footsteps}, {@link #swingDurations}, {@link #transferDurations}, and
    * {@link #finalTransferDuration} should always be set together. Note, that they will only be used if one of the
    * initialize flags is set to {@code true}
    */
   private final RecyclingArrayList<SimpleAdjustableFootstep> footsteps = new RecyclingArrayList<>(SimpleAdjustableFootstep.class);

   /**
    * List of upcoming footstep swing durations that are being executed by the controller. This is of interest to the
    * ICP controller because it might consider n-step capturability or adjust the locations of the upcoming footsteps
    * when needed.
    * <p>
    * The fields {@link #footsteps}, {@link #swingDurations}, {@link #transferDurations}, and
    * {@link #finalTransferDuration} should always be set together. Note, that they will only be used if one of the
    * initialize flags is set to {@code true}
    */
   private final TDoubleArrayList swingDurations = new TDoubleArrayList();

   /**
    * List of upcoming footstep transfer durations that are being executed by the controller. This is of interest to the
    * ICP controller because it might consider n-step capturability or adjust the locations of the upcoming footsteps
    * when needed.
    * <p>
    * The fields {@link #footsteps}, {@link #swingDurations}, {@link #transferDurations}, and
    * {@link #finalTransferDuration} should always be set together. Note, that they will only be used if one of the
    * initialize flags is set to {@code true}
    */
   private final TDoubleArrayList transferDurations = new TDoubleArrayList();

   /**
    * The final transfer duration for a footstep plan. This is a separate field since footsteps only hold the time to
    * transfer to and the time to swing. Hence, the time for the final transfer back to standing is not contained in the
    * list of {@link #footstepTimings}.
    * <p>
    * The fields {@link #footsteps}, {@link #swingDurations}, {@link #transferDurations}, and
    * {@link #finalTransferDuration} should always be set together. Note, that they will only be used if one of the
    * initialize flags is set to {@code true}
    */
   private double finalTransferDuration;

   /**
    * This field can be used to inform the ICP controller that the walking state machine has sped up the swing of the
    * robot. This can happen under disturbances.
    * <p>
    * This value can be set to {@code NaN} or to {@code <= 0.0} to indicate that no swing speed up has been performed.
    */
   private double remainingTimeInSwingUnderDisturbance;

   /**
    * The contact state of the robot. Effectively updates the support polygon for the ICP feedback controller.
    */
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   public void setOmega0(double omega0)
   {
      this.omega0 = omega0;
   }

   public double getOmega0()
   {
      return omega0;
   }

   public void setDesiredCapturePoint(FramePoint2DReadOnly desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public FramePoint2D getDesiredCapturePoint()
   {
      return desiredCapturePoint;
   }

   public void setDesiredCapturePointVelocity(FrameVector2DReadOnly desiredCapturePointVelocity)
   {
      this.desiredCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocity);
   }

   public FrameVector2D getDesiredCapturePointVelocity()
   {
      return desiredCapturePointVelocity;
   }

   @Deprecated // TODO: This should not be coming from the walking controller.
   public void setDesiredCenterOfMassHeightAcceleration(double desiredCoMHeightAcceleration)
   {
      this.desiredCoMHeightAcceleration = desiredCoMHeightAcceleration;
   }

   public double getDesiredCoMHeightAcceleration()
   {
      return desiredCoMHeightAcceleration;
   }

   public void setMinimizeAngularMomentumRateZ(boolean minimizeAngularMomentumRateZ)
   {
      this.minimizeAngularMomentumRateZ = minimizeAngularMomentumRateZ;
   }

   public boolean getMinimizeAngularMomentumRateZ()
   {
      return minimizeAngularMomentumRateZ;
   }

   public void setPerfectCMP(FramePoint2DReadOnly perfectCMP)
   {
      this.perfectCMP.setIncludingFrame(perfectCMP);
   }

   public FramePoint2D getPerfectCMP()
   {
      return perfectCMP;
   }

   public void setPerfectCoP(FramePoint2DReadOnly perfectCoP)
   {
      this.perfectCoP.setIncludingFrame(perfectCoP);
   }

   public FramePoint2D getPerfectCoP()
   {
      return perfectCoP;
   }

   public void setControlHeightWithMomentum(boolean controlHeightWithMomentum)
   {
      this.controlHeightWithMomentum = controlHeightWithMomentum;
   }

   public boolean getControlHeightWithMomentum()
   {
      return controlHeightWithMomentum;
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public RobotSide getSupportSide()
   {
      return supportSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   public void setFromFootsteps(List<Footstep> footsteps)
   {
      this.footsteps.clear();
      for (int i = 0; i < footsteps.size(); i++)
      {
         this.footsteps.add().set(footsteps.get(i));
      }
   }

   public void setFootsteps(List<SimpleAdjustableFootstep> footsteps)
   {
      this.footsteps.clear();
      for (int i = 0; i < footsteps.size(); i++)
      {
         this.footsteps.add().set(footsteps.get(i));
      }
   }

   public RecyclingArrayList<SimpleAdjustableFootstep> getFootsteps()
   {
      return footsteps;
   }

   public void setFromFootstepTimings(List<FootstepTiming> footstepTimings)
   {
      swingDurations.reset();
      transferDurations.reset();
      for (int i = 0; i < footsteps.size(); i++)
      {
         swingDurations.add(footstepTimings.get(i).getSwingTime());
         transferDurations.add(footstepTimings.get(i).getTransferTime());
      }
   }

   public void setSwingDurations(TDoubleArrayList swingDurations)
   {
      this.swingDurations.reset();
      this.swingDurations.addAll(swingDurations);
   }

   public TDoubleArrayList getSwingDurations()
   {
      return swingDurations;
   }

   public void setTransferDurations(TDoubleArrayList transferDurations)
   {
      this.transferDurations.reset();
      this.transferDurations.addAll(transferDurations);
   }

   public TDoubleArrayList getTransferDurations()
   {
      return transferDurations;
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration = finalTransferDuration;
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration;
   }

   public void setInitializeForStanding(boolean initializeForStanding)
   {
      this.initializeForStanding = initializeForStanding;
   }

   public boolean getInitializeForStanding()
   {
      return initializeForStanding;
   }

   public void setInitializeForSingleSupport(boolean initializeForSingleSupport)
   {
      this.initializeForSingleSupport = initializeForSingleSupport;
   }

   public boolean getInitializeForSingleSupport()
   {
      return initializeForSingleSupport;
   }

   public void setInitializeForTransfer(boolean initializeForTransfer)
   {
      this.initializeForTransfer = initializeForTransfer;
   }

   public boolean getInitializeForTransfer()
   {
      return initializeForTransfer;
   }

   public void setRemainingTimeInSwingUnderDisturbance(double remainingTimeInSwingUnderDisturbance)
   {
      this.remainingTimeInSwingUnderDisturbance = remainingTimeInSwingUnderDisturbance;
   }

   public double getRemainingTimeInSwingUnderDisturbance()
   {
      return remainingTimeInSwingUnderDisturbance;
   }

   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.keepCoPInsideSupportPolygon = keepCoPInsideSupportPolygon;
   }

   public boolean getKeepCoPInsideSupportPolygon()
   {
      return keepCoPInsideSupportPolygon;
   }

   public void setContactStateCommand(SideDependentList<PlaneContactStateCommand> contactStateCommands)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.contactStateCommands.get(robotSide).set(contactStateCommands.get(robotSide));
      }
   }

   public SideDependentList<PlaneContactStateCommand> getContactStateCommands()
   {
      return contactStateCommands;
   }

   public void set(LinearMomentumRateControlModuleInput other)
   {
      omega0 = other.omega0;
      desiredCapturePoint.setIncludingFrame(other.desiredCapturePoint);
      desiredCapturePointVelocity.setIncludingFrame(other.desiredCapturePointVelocity);
      perfectCMP.setIncludingFrame(other.perfectCMP);
      perfectCoP.setIncludingFrame(other.perfectCoP);
      controlHeightWithMomentum = other.controlHeightWithMomentum;
      desiredCoMHeightAcceleration = other.desiredCoMHeightAcceleration;
      supportSide = other.supportSide;
      transferToSide = other.transferToSide;
      initializeForStanding = other.initializeForStanding;
      initializeForSingleSupport = other.initializeForSingleSupport;
      initializeForTransfer = other.initializeForTransfer;
      keepCoPInsideSupportPolygon = other.keepCoPInsideSupportPolygon;
      minimizeAngularMomentumRateZ = other.minimizeAngularMomentumRateZ;
      footsteps.clear();
      for (int i = 0; i < other.footsteps.size(); i++)
         footsteps.add().set(other.footsteps.get(i));
      swingDurations.reset();
      swingDurations.addAll(other.swingDurations);
      transferDurations.reset();
      transferDurations.addAll(other.transferDurations);
      finalTransferDuration = other.finalTransferDuration;
      remainingTimeInSwingUnderDisturbance = other.remainingTimeInSwingUnderDisturbance;
      for (RobotSide robotSide : RobotSide.values)
         contactStateCommands.get(robotSide).set(other.contactStateCommands.get(robotSide));
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof LinearMomentumRateControlModuleInput)
      {
         LinearMomentumRateControlModuleInput other = (LinearMomentumRateControlModuleInput) obj;
         if (Double.compare(omega0, other.omega0) != 0)
            return false;
         if (!desiredCapturePoint.equals(other.desiredCapturePoint))
            return false;
         if (!desiredCapturePointVelocity.equals(other.desiredCapturePointVelocity))
            return false;
         if (!perfectCMP.equals(other.perfectCMP))
            return false;
         if (!perfectCoP.equals(other.perfectCoP))
            return false;
         if (controlHeightWithMomentum ^ other.controlHeightWithMomentum)
            return false;
         if (Double.compare(desiredCoMHeightAcceleration, other.desiredCoMHeightAcceleration) != 0)
            return false;
         if (supportSide != other.supportSide)
            return false;
         if (transferToSide != other.transferToSide)
            return false;
         if (initializeForStanding ^ other.initializeForStanding)
            return false;
         if (initializeForSingleSupport ^ other.initializeForSingleSupport)
            return false;
         if (initializeForTransfer ^ other.initializeForTransfer)
            return false;
         if (keepCoPInsideSupportPolygon ^ other.keepCoPInsideSupportPolygon)
            return false;
         if (minimizeAngularMomentumRateZ ^ other.minimizeAngularMomentumRateZ)
            return false;
         if (!footsteps.equals(other.footsteps))
            return false;
         if (!swingDurations.equals(other.swingDurations))
            return false;
         if (!transferDurations.equals(other.transferDurations))
            return false;
         if (Double.compare(finalTransferDuration, other.finalTransferDuration) != 0)
            return false;
         if (Double.compare(remainingTimeInSwingUnderDisturbance, other.remainingTimeInSwingUnderDisturbance) != 0)
            return false;
         for (RobotSide robotSide : RobotSide.values)
         {
            if (!contactStateCommands.get(robotSide).equals(other.contactStateCommands.get(robotSide)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }
}
