package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public class QPObjectiveCommand implements InverseDynamicsCommand<QPObjectiveCommand>
{
   private int commandId;
   private final DMatrixRMaj objective = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj jacobian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj weightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj selectionMatrix = new DMatrixRMaj(1, 1);

   public QPObjectiveCommand()
   {
   }

   @Override
   public void set(QPObjectiveCommand other)
   {
      commandId = other.commandId;
      objective.set(other.objective);
      jacobian.set(other.jacobian);
      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.QP_INPUT;
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   public DMatrixRMaj getObjective()
   {
      return objective;
   }

   public DMatrixRMaj getJacobian()
   {
      return jacobian;
   }

   public DMatrixRMaj getWeightMatrix()
   {
      return weightMatrix;
   }

   public DMatrixRMaj getSelectionMatrix()
   {
      return selectionMatrix;
   }
}
