package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class VirtualWrenchCommand implements InverseDynamicsCommand<VirtualWrenchCommand>
{
   private RigidBody controlledBody;
   private String rigidBodyName;
   private Wrench virtualWrenchAppliedByRigidBody = new Wrench();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

   public VirtualWrenchCommand()
   {
   }

   public void setRigidBody(RigidBody controlledBody)
   {
      this.controlledBody = controlledBody;
      rigidBodyName = controlledBody.getName();
   }

   public void set(RigidBody controlledBody, Wrench virtualWrench)
   {
      setRigidBody(controlledBody);
      virtualWrenchAppliedByRigidBody.set(virtualWrench);
      virtualWrenchAppliedByRigidBody.changeFrame(controlledBody.getBodyFixedFrame());
   }

   public void set(RigidBody controlledBody, Wrench virtualWrench, DenseMatrix64F selectionMatrix)
   {
      set(controlledBody, virtualWrench);
      this.selectionMatrix.set(selectionMatrix);
   }

   public RigidBody getControlledBody()
   {
      return controlledBody;
   }

   public String getControlledBodyName()
   {
      return rigidBodyName;
   }

   public Wrench getVirtualWrench()
   {
      return virtualWrenchAppliedByRigidBody;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public void set(VirtualWrenchCommand other)
   {
      controlledBody = other.controlledBody;
      rigidBodyName = other.rigidBodyName;
      virtualWrenchAppliedByRigidBody.set(other.virtualWrenchAppliedByRigidBody);
      selectionMatrix.set(other.selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.VIRTUAL_WRENCH;
   }
}
