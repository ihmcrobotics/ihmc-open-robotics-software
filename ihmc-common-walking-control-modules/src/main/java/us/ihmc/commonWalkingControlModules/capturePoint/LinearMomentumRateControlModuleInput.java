package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
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
    * Boolean saying whether or not to use the momentum recovery mode. If this is set to yes, a higher momentum rate weight is used.
    */
   private boolean useMomentumRecoveryMode;

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

   private FeedbackControlCommand<?> heightControlCommand;

   @Deprecated // The CoM height control should be moved to the fast thread or this should use the achieved value from the last tick.
   private double desiredCoMHeightAcceleration = 0.0;

   /**
    * Indicates which foot will be in support when stepping. Note, that this will only be used if
    * {@link #initializeForSingleSupport} is set to {@code true}.
    */
   private RobotSide supportSide = null;

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

   public void setUseMomentumRecoveryMode(boolean useMomentumRecoveryMode)
   {
      this.useMomentumRecoveryMode = useMomentumRecoveryMode;
   }

   public boolean getUseMomentumRecoveryMode()
   {
      return useMomentumRecoveryMode;
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

   public void setHeightControlCommand(FeedbackControlCommand<?> heightControlCommand)
   {
      this.heightControlCommand = heightControlCommand;
   }

   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      return heightControlCommand;
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
      useMomentumRecoveryMode = other.useMomentumRecoveryMode;
      desiredCapturePoint.setIncludingFrame(other.desiredCapturePoint);
      desiredCapturePointVelocity.setIncludingFrame(other.desiredCapturePointVelocity);
      perfectCMP.setIncludingFrame(other.perfectCMP);
      perfectCoP.setIncludingFrame(other.perfectCoP);
      controlHeightWithMomentum = other.controlHeightWithMomentum;
      desiredCoMHeightAcceleration = other.desiredCoMHeightAcceleration;
      supportSide = other.supportSide;
      initializeForStanding = other.initializeForStanding;
      initializeForSingleSupport = other.initializeForSingleSupport;
      initializeForTransfer = other.initializeForTransfer;
      keepCoPInsideSupportPolygon = other.keepCoPInsideSupportPolygon;
      minimizeAngularMomentumRateZ = other.minimizeAngularMomentumRateZ;
      heightControlCommand = other.heightControlCommand;
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
         if (useMomentumRecoveryMode ^ other.useMomentumRecoveryMode)
            return false;
         if (Double.compare(desiredCoMHeightAcceleration, other.desiredCoMHeightAcceleration) != 0)
            return false;
         if (supportSide != other.supportSide)
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
         if (heightControlCommand != null && !heightControlCommand.equals(other.heightControlCommand))
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
