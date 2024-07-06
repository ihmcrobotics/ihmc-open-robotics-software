package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public class CoPObjectiveCalculator
{
   private final DMatrixRMaj fzRow = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj singleCopRow = new DMatrixRMaj(0, 0);

   public void computeTask(DMatrixRMaj wrenchJacobianInPlaneFrame,
                           FramePoint2DReadOnly desiredCoPInPlaneFrame,
                           int rhoSize,
                           DMatrix jacobianToPack,
                           DMatrix objectiveToPack)
   {
      fzRow.reshape(1, rhoSize);
      singleCopRow.reshape(1, rhoSize);

      int fzIndex = 5;
      CommonOps_DDRM.extractRow(wrenchJacobianInPlaneFrame, fzIndex, fzRow);

      // [x_cop * J_fz + J_ty] * rho == 0
      int tauYIndex = 1;
      CommonOps_DDRM.extractRow(wrenchJacobianInPlaneFrame, tauYIndex, singleCopRow);
      CommonOps_DDRM.add(desiredCoPInPlaneFrame.getX(), fzRow, 1.0, singleCopRow, singleCopRow);
      CommonOps_DDRM.insert(singleCopRow, jacobianToPack, 0, 0);

      // [y_cop * J_fz - J_tx] * rho == 0
      int tauXIndex = 0;
      CommonOps_DDRM.extractRow(wrenchJacobianInPlaneFrame, tauXIndex, singleCopRow);
      CommonOps_DDRM.add(desiredCoPInPlaneFrame.getY(), fzRow, -1.0, singleCopRow, singleCopRow);
      CommonOps_DDRM.insert(singleCopRow, jacobianToPack, 1, 0);

      objectiveToPack.zero();
   }
}
