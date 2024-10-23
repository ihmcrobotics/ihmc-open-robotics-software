package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

/**
 * Command is a cost function to go directly into QP. It is of the form
 * <p>
 * w (0.5 u<sup>T</sup> Q u + q u),
 * where
 * u = Jx - b
 * <p>
 * where u is the momentum rate of change
 */
public class QPCostCommand implements InverseDynamicsCommand<QPCostCommand>
{
   private int commandId;

   /**
    * Q in the above equations
    */
   private final DMatrixRMaj costHessian = new DMatrixRMaj(1, 1);

   /**
    * q in the above equations
    */
   private final DMatrixRMaj costGradient = new DMatrixRMaj(1, 1);

   /**
    * J in the above equations
    */
   private final DMatrixRMaj stateJacobian = new DMatrixRMaj(1, 1);

   /**
    * b in the above equation
    */
   private final DMatrixRMaj stateObjective = new DMatrixRMaj(1, 1);

   /**
    * w in the above equation
    */
   private double weight = 1.0;

   public QPCostCommand()
   {
   }

   @Override
   public void set(QPCostCommand other)
   {
      commandId = other.commandId;
      costHessian.set(other.costHessian);
      costGradient.set(other.costGradient);
      stateJacobian.set(other.stateJacobian);
      stateObjective.set(other.stateObjective);
      weight = other.weight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setCostHessian(DMatrixRMaj costHessian)
   {
      this.costHessian.set(costHessian);
   }

   public void setCostGradient(DMatrixRMaj costGradient)
   {
      this.costGradient.set(costGradient);
   }

   public void setStateJacobian(DMatrixRMaj stateJacobian)
   {
      this.stateJacobian.set(stateJacobian);
   }

   public void setStateObjective(DMatrixRMaj stateObjective)
   {
      this.stateObjective.set(stateObjective);
   }

   public double getWeight()
   {
      return weight;
   }

   public DMatrixRMaj getCostHessian()
   {
      return costHessian;
   }

   public DMatrixRMaj getCostGradient()
   {
      return costGradient;
   }

   public DMatrixRMaj getStateJacobian()
   {
      return stateJacobian;
   }

   public DMatrixRMaj getStateObjective()
   {
      return stateObjective;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.QP_COST;
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

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof QPCostCommand)
      {
         QPCostCommand other = (QPCostCommand) object;

         if (commandId != other.commandId)
            return false;
         if (!MatrixFeatures_DDRM.isEquals(costHessian, other.costHessian))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(costGradient, other.costGradient))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(stateJacobian, other.stateJacobian))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(stateObjective, other.stateObjective))
            return false;
         if (weight != other.weight)
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
