package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CenterOfPressureCommand implements InverseDynamicsCommand<CenterOfPressureCommand>
{
   private RigidBody contactingRigidBody;
   private String contactingRigidBodyName;
   private final Vector2d weightInSoleFrame = new Vector2d();
   private final Point2d desiredCoPInSoleFrame = new Point2d();

   public CenterOfPressureCommand()
   {
   }

   @Override
   public void set(CenterOfPressureCommand other)
   {
      this.weightInSoleFrame.set(other.weightInSoleFrame);
      this.desiredCoPInSoleFrame.set(other.desiredCoPInSoleFrame);

      this.contactingRigidBody = other.getContactingRigidBody();
      this.contactingRigidBodyName = other.contactingRigidBodyName;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.CENTER_OF_PRESSURE;
   }

   public void setContactingRigidBody(RigidBody contactingRigidBody)
   {
      this.contactingRigidBody = contactingRigidBody;
      this.contactingRigidBodyName = contactingRigidBody.getName();
   }

   public void setWeight(Vector2d weightInSoleFrame)
   {
      this.weightInSoleFrame.set(weightInSoleFrame);
   }

   public void setDesiredCoP(Point2d desiredCoPInSoleFrame)
   {
      this.desiredCoPInSoleFrame.set(desiredCoPInSoleFrame);
   }

   public Point2d getDesiredCoPInSoleFrame()
   {
      return desiredCoPInSoleFrame;
   }

   public Vector2d getWeightInSoleFrame()
   {
      return weightInSoleFrame;
   }

   public String getContactingRigidBodyName()
   {
      return contactingRigidBodyName;
   }

   public RigidBody getContactingRigidBody()
   {
      return contactingRigidBody;
   }

}
