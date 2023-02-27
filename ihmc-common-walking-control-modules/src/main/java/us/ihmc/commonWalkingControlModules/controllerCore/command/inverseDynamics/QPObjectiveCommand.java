package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;

/**
 * Command is a cost function to go directly into QP. It is of the form
 *
 * 0.5 (S J x - S b)<sup>T</sup> S<sup>T</sup> Q S (S J x - S b)
 *
 * where x is the variable vector specified in the {@link us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore},
 * J is the function Jacobian, and b is the objective.
 *
 * This enables setting custom cost terms and commands for the whole-body controller.
 */
public class QPObjectiveCommand implements InverseDynamicsCommand<QPObjectiveCommand>, VirtualModelControlCommand<QPObjectiveCommand>
{
   private int commandId;
   /**
    * b in the above equations
    */
   private final DMatrixRMaj objective = new DMatrixRMaj(1, 1);
   /**
    * J in the above equations. Jacobian that maps from the variable space to the objective space.
    */
   private final DMatrixRMaj jacobian = new DMatrixRMaj(1, 1);
   /**
    * Q in the above equations
    */
   private final DMatrixRMaj weightMatrix = new DMatrixRMaj(1, 1);
   /**
    * S in the above equations
    */
   private final DMatrixRMaj selectionMatrix = new DMatrixRMaj(1, 1);

   /**
    * Boolean that indicates whether or not this command should be added in the nullspace of the other, primary commands.
    */
   private boolean doNullspaceProjection = false;

   public QPObjectiveCommand()
   {
   }

   public void setDoNullSpaceProjection(boolean flag)
   {
      doNullspaceProjection = flag;
   }

   /**
    * If this returns true, this task is added to the optimization in the nullspace of the other, primary commands.
    */
   public boolean isNullspaceProjected()
   {
      return doNullspaceProjection;
   }

   /**
    * Reshapes the task. Note that if the number of degrees of freedom is not equal to the number of free variables in the optimization, this will result in an
    * exception being thrown.
    *
    * @param numberOfRows This is the size of the objective for this command
    * @param numberOfDoFs This is the number of degrees of freedom in the optimization.
    */
   public void reshape(int numberOfRows, int numberOfDoFs)
   {
      objective.reshape(numberOfRows, 1);
      jacobian.reshape(numberOfRows, numberOfDoFs);
      weightMatrix.reshape(numberOfRows, numberOfRows);
      selectionMatrix.reshape(numberOfRows, numberOfRows);
      CommonOps_DDRM.setIdentity(selectionMatrix);
   }


   @Override
   public void set(QPObjectiveCommand other)
   {
      commandId = other.commandId;
      objective.set(other.objective);
      jacobian.set(other.jacobian);
      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
      doNullspaceProjection = other.doNullspaceProjection;
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

   /**
    * Gets this command's Jacobian matrix.
    * The Jacobian matrix is used to map from the DoFs (Degrees of Freedom) in the cost function to the
    * desired command.
    *
    * @return The Jacobian matrix to be used in the command.
    */
   public DMatrixRMaj getJacobian()
   {
      return jacobian;
   }

   /**
    * Gets this command's weight matrix.
    * The weight matrix is used to prioritize the relative weight of this command in the cost function.
    *
    * @return The weight matrix to be used in the command.
    */
   public DMatrixRMaj getWeightMatrix()
   {
      return weightMatrix;
   }

   /**
    * Gets this command's selection matrix.
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the command that are to
    * be used to achieve the desired objective. It is initialized such that the controller will by
    * default control all the DoFs.
    *
    * @return The selection matrix to be used in the command.
    */
   public DMatrixRMaj getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof QPObjectiveCommand)
      {
         QPObjectiveCommand other = (QPObjectiveCommand) object;

         if (commandId != other.commandId)
            return false;
         if (!MatrixFeatures_DDRM.isEquals(weightMatrix, other.weightMatrix))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(selectionMatrix, other.selectionMatrix))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(objective, other.objective))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(jacobian, other.jacobian))
            return false;
         if (doNullspaceProjection != other.doNullspaceProjection)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
