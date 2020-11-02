package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * Command is a cost function to go directly into QP. It is of the form
 *
 * 0.5 u<sup>T</sup> Q u + q u
 *
 * where u is the momentum rate of change
 */
public class LinearMomentumRateCostCommand implements InverseDynamicsCommand<LinearMomentumRateCostCommand>
{
   private int commandId;

   /**
    * Q in the above equations
    */
   private final DMatrixRMaj momentumRateHessian = new DMatrixRMaj(Momentum.SIZE, Momentum.SIZE);

   /**
    * q in the above equations
    */
   private final DMatrixRMaj momentumRateGradient = new DMatrixRMaj(1, Momentum.SIZE);

   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   public LinearMomentumRateCostCommand()
   {
      weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
      selectionMatrix.clearAngularSelection();
   }

   @Override
   public void set(LinearMomentumRateCostCommand other)
   {
      commandId = other.commandId;
      momentumRateHessian.set(other.momentumRateHessian);
      momentumRateGradient.set(other.momentumRateGradient);
      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.resetSelection();
      selectionMatrix.clearAngularSelection();
   }

   public void setSelectionMatrixForLinearXYControl()
   {
      selectionMatrix.resetSelection();
      selectionMatrix.clearAngularSelection();
      selectionMatrix.selectLinearZ(false);
   }

   public void setWeight(double weight)
   {
      weightMatrix.getLinearPart().setWeights(weight, weight, weight);
   }

   public void setWeights(double linearX, double linearY, double linearZ)
   {
      weightMatrix.getLinearPart().setWeights(linearX, linearY, linearZ);
   }

   public void setWeights(Tuple3DReadOnly linear)
   {
      weightMatrix.getLinearPart().setWeights(linear.getX(), linear.getY(), linear.getZ());
   }

   public void getWeightMatrix(DMatrixRMaj weightMatrixToPack)
   {
      weightMatrixToPack.reshape(Momentum.SIZE, Momentum.SIZE);
      weightMatrix.getFullWeightMatrixInFrame(ReferenceFrame.getWorldFrame(), weightMatrixToPack);
   }

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   public void getSelectionMatrix(ReferenceFrame destinationFrame, DMatrixRMaj selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, selectionMatrixToPack);
   }

   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setLinearMomentumRateHessian(DMatrixRMaj linearMomentumRateHessian)
   {
      MatrixTools.setMatrixBlock(this.momentumRateHessian, 3, 3, linearMomentumRateHessian, 0, 0, 3, 3, 1.0);
   }

   public void setLinearMomentumRateGradient(DMatrixRMaj linearMomentumRateGradient)
   {
      MatrixTools.setMatrixBlock(this.momentumRateGradient, 0, 3, linearMomentumRateGradient, 0, 0, 1, 3, 1.0);

      this.momentumRateGradient.set(momentumRateGradient);
   }

   public DMatrixRMaj getMomentumRateHessian()
   {
      return momentumRateHessian;
   }

   public DMatrixRMaj getMomentumRateGradient()
   {
      return momentumRateGradient;
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
         if (!MatrixFeatures_DDRM.isEquals(momentumRateHessian, other.momentumRateHessian))
            return false;
         if (!MatrixFeatures_DDRM.isEquals(momentumRateGradient, other.momentumRateGradient))
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
