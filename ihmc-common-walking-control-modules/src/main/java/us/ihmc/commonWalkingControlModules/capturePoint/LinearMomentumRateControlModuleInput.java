package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
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

   /**
    * Specifies whether the momentum module should use the pelvis height control command or the CoM height control command.
    */
   private boolean usePelvisHeightCommand;

   /**
    * Contains the feedback command information for the center of mass height. Used to compute the necessary vertical momentum.
    */
   private final CenterOfMassFeedbackControlCommand comHeightControlCommand = new CenterOfMassFeedbackControlCommand();

   /**
    * Contains the feedback command information for the pelvis height. Used to compute the necessary vertical momentum.
    */
   private final PointFeedbackControlCommand pelvisHeightControlCommand = new PointFeedbackControlCommand();

   /**
    * Flag that indicates the ICP planner has just transitioned in contact state.
    * <p>
    */
   private boolean initializeOnStateChange;

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

   public void setUsePelvisHeightCommand(boolean usePelvisHeightCommand)
   {
      this.usePelvisHeightCommand = usePelvisHeightCommand;
   }

   public void setPelvisHeightControlCommand(PointFeedbackControlCommand heightControlCommand)
   {
      this.pelvisHeightControlCommand.set(heightControlCommand);
   }

   public void setCenterOfMassHeightControlCommand(CenterOfMassFeedbackControlCommand heightControlCommand)
   {
      this.comHeightControlCommand.set(heightControlCommand);
   }

   public boolean getUsePelvisHeightCommand()
   {
      return usePelvisHeightCommand;
   }

   public PointFeedbackControlCommand getPelvisHeightControlCommand()
   {
      return pelvisHeightControlCommand;
   }

   public CenterOfMassFeedbackControlCommand getCenterOfMassHeightControlCommand()
   {
      return comHeightControlCommand;
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

   public void setInitializeOnStateChange(boolean initializeOnStateChange)
   {
      this.initializeOnStateChange = initializeOnStateChange;
   }

   public boolean getInitializeOnStateChange()
   {
      return initializeOnStateChange;
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
      initializeOnStateChange = other.initializeOnStateChange;
      keepCoPInsideSupportPolygon = other.keepCoPInsideSupportPolygon;
      minimizeAngularMomentumRateZ = other.minimizeAngularMomentumRateZ;
      setUsePelvisHeightCommand(other.getUsePelvisHeightCommand());
      setPelvisHeightControlCommand(other.getPelvisHeightControlCommand());
      setCenterOfMassHeightControlCommand(other.getCenterOfMassHeightControlCommand());
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
         if (initializeOnStateChange ^ other.initializeOnStateChange)
            return false;
         if (keepCoPInsideSupportPolygon ^ other.keepCoPInsideSupportPolygon)
            return false;
         if (minimizeAngularMomentumRateZ ^ other.minimizeAngularMomentumRateZ)
            return false;
         if (usePelvisHeightCommand ^ other.usePelvisHeightCommand)
            return false;
         if (!pelvisHeightControlCommand.equals(other.pelvisHeightControlCommand))
            return false;
         if (!comHeightControlCommand.equals(other.comHeightControlCommand))
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
