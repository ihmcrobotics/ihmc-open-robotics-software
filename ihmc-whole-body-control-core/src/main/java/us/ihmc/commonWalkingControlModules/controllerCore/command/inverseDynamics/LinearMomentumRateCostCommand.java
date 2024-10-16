package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

/**
 * Command is a cost function to go directly into QP. It is of the form
 *
 * w ( 0.5 (S u) <sup>T</sup> (S Q S<sup>T</sup>) (S u ) + q u )
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

   /**
    * w in the above equations
    */
   private double weight = 1.0;

   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   public LinearMomentumRateCostCommand()
   {
      selectionMatrix.clearAngularSelection();
   }

   @Override
   public void set(LinearMomentumRateCostCommand other)
   {
      commandId = other.commandId;
      momentumRateHessian.set(other.momentumRateHessian);
      momentumRateGradient.set(other.momentumRateGradient);
      weight = other.weight;
      selectionMatrix.set(other.selectionMatrix);
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public double getWeight()
   {
      return weight;
   }

   public void getSelectionMatrix(ReferenceFrame destinationFrame, DMatrixRMaj selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, selectionMatrixToPack);
   }

   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector that
    * are to be controlled. It is initialized such that the controller will by default control all the
    * end-effector DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    *
    * @param selectionMatrix the selection matrix to copy data from. Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setMomentumRateHessian(DMatrixRMaj momentumRateHessian)
   {
      this.momentumRateHessian.set(momentumRateHessian);
   }

   public void setMomentumRateGradient(DMatrixRMaj momentumRateGradient)
   {
      this.momentumRateGradient.set(momentumRateGradient);
   }

   public void setLinearMomentumRateHessian(DMatrixRMaj linearMomentumRateHessian)
   {
      MatrixTools.setMatrixBlock(this.momentumRateHessian, 3, 3, linearMomentumRateHessian, 0, 0, 3, 3, 1.0);
   }

   public void setLinearMomentumRateGradient(DMatrixRMaj linearMomentumRateGradient)
   {
      MatrixTools.setMatrixBlock(this.momentumRateGradient, 0, 3, linearMomentumRateGradient, 0, 0, 1, 3, 1.0);
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
         if (weight != other.weight)
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
