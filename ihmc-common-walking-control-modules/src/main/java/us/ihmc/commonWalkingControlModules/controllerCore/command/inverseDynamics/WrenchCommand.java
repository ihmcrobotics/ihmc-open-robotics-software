package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

public class WrenchCommand implements InverseDynamicsCommand<WrenchCommand>
{
   private ConstraintType constraintType;

   private RigidBody rigidBody;
   private final Wrench wrench = new Wrench();

   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   public WrenchCommand()
   {
   }

   public WrenchCommand(ConstraintType constraintType)
   {
      setConstraintType(constraintType);
   }

   public void setRigidBody(RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public Wrench getWrench()
   {
      return wrench;
   }

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public void set(WrenchCommand other)
   {
      this.constraintType = other.constraintType;
      this.rigidBody = other.rigidBody;
      this.wrench.set(other.wrench);
      this.weightMatrix.set(other.weightMatrix);
      this.selectionMatrix.set(other.selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.WRENCH_OBJECTIVE;
   }
}
