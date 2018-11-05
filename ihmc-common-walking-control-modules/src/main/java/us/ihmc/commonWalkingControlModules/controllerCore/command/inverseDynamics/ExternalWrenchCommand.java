package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ExternalWrenchCommand implements InverseDynamicsCommand<ExternalWrenchCommand>, VirtualModelControlCommand<ExternalWrenchCommand>
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

   public void set(RigidBody rigidBody, WrenchReadOnly externalWrench)
   {
      setRigidBody(rigidBody);
      externalWrenchAppliedOnRigidBody.setIncludingFrame(externalWrench);
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

   public WrenchReadOnly getExternalWrench()
   {
      return externalWrenchAppliedOnRigidBody;
   }

   @Override
   public void set(ExternalWrenchCommand other)
   {
      rigidBody = other.rigidBody;
      rigidBodyName = other.rigidBodyName;
      externalWrenchAppliedOnRigidBody.setIncludingFrame(other.externalWrenchAppliedOnRigidBody);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.EXTERNAL_WRENCH;
   }
}
