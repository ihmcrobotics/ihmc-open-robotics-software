package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class ExternalWrenchCommand implements InverseDynamicsCommand<ExternalWrenchCommand>, VirtualModelControlCommand<ExternalWrenchCommand>
{
   private RigidBodyBasics rigidBody;
   private final Wrench externalWrenchAppliedOnRigidBody = new Wrench();

   public ExternalWrenchCommand()
   {
   }

   public void setRigidBody(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;
   }

   public void set(RigidBodyBasics rigidBody, WrenchReadOnly externalWrench)
   {
      setRigidBody(rigidBody);
      externalWrenchAppliedOnRigidBody.setIncludingFrame(externalWrench);
      externalWrenchAppliedOnRigidBody.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public WrenchBasics getExternalWrench()
   {
      return externalWrenchAppliedOnRigidBody;
   }

   @Override
   public void set(ExternalWrenchCommand other)
   {
      rigidBody = other.rigidBody;
      externalWrenchAppliedOnRigidBody.setIncludingFrame(other.externalWrenchAppliedOnRigidBody);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.EXTERNAL_WRENCH;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ExternalWrenchCommand)
      {
         ExternalWrenchCommand other = (ExternalWrenchCommand) object;
         if (rigidBody != other.rigidBody)
            return false;
         if (!externalWrenchAppliedOnRigidBody.equals(other.externalWrenchAppliedOnRigidBody))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": body: " + rigidBody + ", wrench: " + externalWrenchAppliedOnRigidBody;
   }
}
