package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class ExternalWrenchCommand extends InverseDynamicsCommand<ExternalWrenchCommand>
{
   private RigidBody rigidBody;
   private final Wrench externalWrenchAppliedOnRigidBody = new Wrench();

   public ExternalWrenchCommand()
   {
      super(InverseDynamicsCommandType.EXTERNAL_WRENCH);
   }

   public void set(RigidBody rigidBody, Wrench externalWrench)
   {
      this.rigidBody = rigidBody;
      externalWrenchAppliedOnRigidBody.set(externalWrench);
      externalWrenchAppliedOnRigidBody.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public Wrench getExternalWrench()
   {
      return externalWrenchAppliedOnRigidBody;
   }

   @Override
   public void set(ExternalWrenchCommand other)
   {
      rigidBody = other.rigidBody;
      externalWrenchAppliedOnRigidBody.set(other.externalWrenchAppliedOnRigidBody);
   }
}
