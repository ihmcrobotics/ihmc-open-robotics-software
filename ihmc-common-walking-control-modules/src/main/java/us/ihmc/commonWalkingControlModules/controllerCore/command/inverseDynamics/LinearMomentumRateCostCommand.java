package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

/**
 * Command is a cost function to go directly into QP. It is of the form
 *
 * u<sup>T</sup> Q u + q u
 *
 * where u is the linear momentum rate of change
 */
public class LinearMomentumRateCostCommand implements InverseDynamicsCommand<LinearMomentumRateCostCommand>
{
   private int commandId;

   /**
    * Q in the above equations
    */
   private final DMatrixRMaj linearMomentumHessian = new DMatrixRMaj(3, 3);

   /**
    * q in the above equations
    */
   private final DMatrixRMaj linearMomentumGradient = new DMatrixRMaj(1, 3);

   private final WeightMatrix3D weightMatrix = new WeightMatrix3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   public LinearMomentumRateCostCommand()
   {
      weightMatrix.setWeights(0.0, 0.0, 0.0);
   }

   @Override
   public void set(LinearMomentumRateCostCommand other)
   {
      commandId = other.commandId;
      linearMomentumHessian.set(other.linearMomentumHessian);
      linearMomentumGradient.set(other.linearMomentumGradient);
      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.resetSelection();
   }

   public void setSelectionMatrixForLinearXYControl()
   {
      selectionMatrix.resetSelection();
      selectionMatrix.selectZAxis(false);
   }

   public void setWeight(double weight)
   {
      weightMatrix.setWeights(weight, weight, weight);
   }

   public void setWeights(double linearX, double linearY, double linearZ)
   {
      weightMatrix.setWeights(linearX, linearY, linearZ);
   }

   public void setWeights(Tuple3DReadOnly linear)
   {
      weightMatrix.setWeights(linear.getX(), linear.getY(), linear.getZ());
   }

   public void getWeightMatrix(DMatrixRMaj weightMatrixToPack)
   {
      weightMatrixToPack.reshape(3, 3);
      weightMatrix.getFullWeightMatrixInFrame(ReferenceFrame.getWorldFrame(), weightMatrixToPack);
   }

   public WeightMatrix3D getWeightMatrix()
   {
      return weightMatrix;
   }

   public void getSelectionMatrix(ReferenceFrame destinationFrame, DMatrixRMaj selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   public void getSelectionMatrix(SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setLinearMomentumHessian(DMatrix linearMomentumHessian)
   {
      this.linearMomentumHessian.set(linearMomentumHessian);
   }

   public void setLinearMomentumGradient(DMatrix linearMomentumGradient)
   {
      this.linearMomentumGradient.set(linearMomentumGradient);
   }

   public DMatrixRMaj getLinearMomentumHessian()
   {
      return linearMomentumHessian;
   }

   public DMatrixRMaj getLinearMomentumGradient()
   {
      return linearMomentumGradient;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM_COST;
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
      else if (object instanceof LinearMomentumRateCostCommand)
      {
         LinearMomentumRateCostCommand other = (LinearMomentumRateCostCommand) object;

         if (commandId != other.commandId)
            return false;
         if (!MatrixFeatures_DDRM.isEquals(linearMomentumHessian, other.linearMomentumHessian))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(linearMomentumGradient, other.linearMomentumGradient))
            return false;
         if (!weightMatrix.equals(other.weightMatrix))
            return false;
         if (!selectionMatrix.equals(other.selectionMatrix))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

}
