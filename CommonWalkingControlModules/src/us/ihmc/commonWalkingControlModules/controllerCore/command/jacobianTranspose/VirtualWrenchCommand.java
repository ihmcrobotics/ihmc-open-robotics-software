package us.ihmc.commonWalkingControlModules.controllerCore.command.jacobianTranspose;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class VirtualWrenchCommand implements JacobianTransposeCommand<VirtualWrenchCommand>
{

   private RigidBody rigidBody;
   private String rigidBodyName;
   private final Wrench virtualWrenchDesiredOnRigidBody = new Wrench();

   public VirtualWrenchCommand()
   {
   }

   public void setRigidBody(RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
      rigidBodyName = rigidBody.getName();
   }

   public void set(RigidBody rigidBody, Wrench virtualWrench)
   {
      setRigidBody(rigidBody);
      virtualWrenchDesiredOnRigidBody.set(virtualWrench);
      virtualWrenchDesiredOnRigidBody.changeFrame(rigidBody.getBodyFixedFrame());
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
      return virtualWrenchDesiredOnRigidBody;
   }

   @Override
   public void set(VirtualWrenchCommand other)
   {
      rigidBody = other.rigidBody;
      rigidBodyName = other.rigidBodyName;
      virtualWrenchDesiredOnRigidBody.set(other.virtualWrenchDesiredOnRigidBody);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.VIRTUAL_WRENCH;
   }
}
