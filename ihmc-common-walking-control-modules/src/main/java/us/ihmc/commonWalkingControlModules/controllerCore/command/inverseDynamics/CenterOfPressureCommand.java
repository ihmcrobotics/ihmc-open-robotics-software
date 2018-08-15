package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CenterOfPressureCommand implements InverseDynamicsCommand<CenterOfPressureCommand>, VirtualModelControlCommand<CenterOfPressureCommand>
{
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   private RigidBody contactingRigidBody;

   private final FrameVector2D weight = new FrameVector2D();
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
