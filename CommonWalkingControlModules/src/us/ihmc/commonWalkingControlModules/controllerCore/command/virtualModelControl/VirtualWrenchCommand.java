package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class VirtualWrenchCommand implements InverseDynamicsCommand<VirtualWrenchCommand>
{
   private RigidBody rigidBody;
   private String rigidBodyName;
   private final Wrench virtualWrenchAppliedByRigidBody = new Wrench();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F();

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
      virtualWrenchAppliedByRigidBody.set(virtualWrench);
      virtualWrenchAppliedByRigidBody.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public void set(RigidBody rigidBody, Wrench virtualWrench, DenseMatrix64F selectionMatrix)
   {
      set(rigidBody, virtualWrench);
      this.selectionMatrix.set(selectionMatrix);
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public String getRigidBodyName()
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
      rigidBody = other.rigidBody;
      rigidBodyName = other.rigidBodyName;
      virtualWrenchAppliedByRigidBody.set(other.virtualWrenchAppliedByRigidBody);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.VIRTUAL_WRENCH;
   }
}
