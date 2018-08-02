package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

public class WrenchObjectiveCommand implements InverseDynamicsCommand<WrenchObjectiveCommand>
{
   private RigidBody rigidBody;
   private final Wrench wrench = new Wrench();

   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   public void setRigidBody(RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
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
   public void set(WrenchObjectiveCommand other)
   {
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
