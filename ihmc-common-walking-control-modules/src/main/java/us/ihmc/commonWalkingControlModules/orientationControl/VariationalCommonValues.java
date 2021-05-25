package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.matrixlib.MatrixTools;

public class VariationalCommonValues
{
   private final DMatrixRMaj Q = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RInverse = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaInverse = new DMatrixRMaj(3, 3);

   public void computeCostMatrices(double Qr, double Qw, double r)
   {
      for (int i = 0; i < 3; i++)
      {
         Q.set(i, i, Qr);
         R.set(i, i, r);
      }
      for (int i = 3; i < 6; i++)
         Q.set(i, i, Qw);

      UnrolledInverseFromMinor_DDRM.inv3(R, RInverse, 1.0);
   }

}
