package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class ExternalWrenchCommand implements InverseDynamicsCommand<ExternalWrenchCommand>
{
   private RigidBody rigidBody;
   private String rigidBodyName;
   private final Wrench externalWrenchAppliedOnRigidBody = new Wrench();

   public ExternalWrenchCommand()
   {
   }

   public void setRigidBody(RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
      rigidBodyName = rigidBody.getName();
   }

   public void set(RigidBody rigidBody, Wrench externalWrench)
   {
      setRigidBody(rigidBody);
      externalWrenchAppliedOnRigidBody.set(externalWrench);
      externalWrenchAppliedOnRigidBody.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public String getRigidBodyName()
   {
      return rigidBodyName;
   }

   public Wrench getExternalWrench()
   {
      return externalWrenchAppliedOnRigidBody;
   }

   @Override
   public void set(ExternalWrenchCommand other)
   {
      rigidBody = other.rigidBody;
      rigidBodyName = other.rigidBodyName;
      externalWrenchAppliedOnRigidBody.set(other.externalWrenchAppliedOnRigidBody);
   }

   @Override
   public InverseDynamicsCommandType getCommandType()
   {
      return InverseDynamicsCommandType.EXTERNAL_WRENCH;
   }
}
