package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * A command that can be used to send center of pressure objectives or constraints to the controller core.
 * <p>
 * If a rigid body is specified in this command the center of pressure command is applied to that body. E.g. the center
 * of pressure of a single foothold can be commanded to a specific position within that foothold. If the body is set to
 * {@code null} then this command will be for the over all robot center of pressure.
 * </p>
 * <p>
 * The center of pressure in this command has an associated frame. Note, that the controller core will use this
 * definition of the CoP for this command: the CoP is the location in the x-y plane of the provided frame at which the
 * x and y torques of the wrench exerted by this body are zero.
 * </p>
 */
public class CenterOfPressureCommand implements InverseDynamicsCommand<CenterOfPressureCommand>, VirtualModelControlCommand<CenterOfPressureCommand>
{
   /**
    * The constraint type of this command can be set to either {@link ConstraintType#OBJECTIVE} or
    * {@link ConstraintType#EQUALITY}. Inequality constraints are not supported here.
    */
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   /**
    * The rigid body that this command if meant for. If set to {@code null} the command will be used for
    * the over all robot CoP.
    */
   private RigidBody contactingRigidBody;

   /**
    * The command weight. Does not need to be expressed in the same frame as the desired CoP but
    * it is recommended that the frames share the x-y plane.
    */
   private final FrameVector2D weight = new FrameVector2D();

   /**
    * The desired CoP. This object also provides the frame for the command: the wrench exerted by the
    * contacting body in this frame will have zero z and y torque if expressed at the CoP location.
    */
   private final FramePoint2D desiredCoP = new FramePoint2D();

   @Override
   public void set(CenterOfPressureCommand other)
   {
      this.constraintType = other.constraintType;
      this.contactingRigidBody = other.contactingRigidBody;
      this.weight.setIncludingFrame(other.weight);
      this.desiredCoP.setIncludingFrame(other.desiredCoP);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.CENTER_OF_PRESSURE;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public void setContactingRigidBody(RigidBody contactingRigidBody)
   {
      this.contactingRigidBody = contactingRigidBody;
   }

   public void setWeight(FrameVector2DReadOnly weightInSoleFrame)
   {
      this.weight.setIncludingFrame(weightInSoleFrame);
   }

   public void setDesiredCoP(FramePoint2DReadOnly desiredCoPInSoleFrame)
   {
      this.desiredCoP.setIncludingFrame(desiredCoPInSoleFrame);
   }

   public FramePoint2DReadOnly getDesiredCoP()
   {
      return desiredCoP;
   }

   public FrameVector2DReadOnly getWeight()
   {
      return weight;
   }

   public RigidBody getContactingRigidBody()
   {
      return contactingRigidBody;
   }

}
