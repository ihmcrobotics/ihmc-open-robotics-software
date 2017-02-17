package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CenterOfPressureCommand implements InverseDynamicsCommand<CenterOfPressureCommand>
{
   private RigidBody contactingRigidBody;
   private String contactingRigidBodyName;
   private final Vector2D weightInSoleFrame = new Vector2D();
   private final Point2D desiredCoPInSoleFrame = new Point2D();

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

   public void setWeight(Vector2D weightInSoleFrame)
   {
      this.weightInSoleFrame.set(weightInSoleFrame);
   }

   public void setDesiredCoP(Point2D desiredCoPInSoleFrame)
   {
      this.desiredCoPInSoleFrame.set(desiredCoPInSoleFrame);
   }

   public Point2D getDesiredCoPInSoleFrame()
   {
      return desiredCoPInSoleFrame;
   }

   public Vector2D getWeightInSoleFrame()
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
