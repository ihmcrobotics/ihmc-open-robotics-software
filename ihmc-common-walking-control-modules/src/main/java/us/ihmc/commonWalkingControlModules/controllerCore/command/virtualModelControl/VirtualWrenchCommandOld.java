package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class VirtualWrenchCommandOld implements VirtualModelControlCommand<VirtualWrenchCommandOld>
{
   private RigidBody endEffector;
   private final Wrench virtualWrenchAppliedByRigidBody = new Wrench();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

   public VirtualWrenchCommandOld()
   {
   }

   public void setRigidBody(RigidBody controlledBody)
   {
      this.endEffector = controlledBody;
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

   /** {@inheritDoc} */
   public RigidBody getEndEffector()
   {
      return endEffector;
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
   public void set(VirtualWrenchCommandOld other)
   {
      endEffector = other.endEffector;
      virtualWrenchAppliedByRigidBody.set(other.virtualWrenchAppliedByRigidBody);
      selectionMatrix.set(other.selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.VIRTUAL_WRENCH;
   }
}
