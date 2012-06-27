package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.fed.fs.fpl.optimization.Lmdif_fcn;
import us.fed.fs.fpl.optimization.Minpack_f77;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public abstract class MomentumOptimizer implements Lmdif_fcn
{
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DenseMatrix64F jointVelocitiesMatrix;
   private final DenseMatrix64F jointAccelerationsMatrix;
   private final DenseMatrix64F centroidalMomentumErrorMatrix = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F aQdd;
   private final DenseMatrix64F adQd;

   private final int m;
   private final int n;
   private final double[] x;
   private final double[] fvec;
   private final double tol = 1e-9;
   private final int[] info = new int[2];

   private final double controlDT;

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;

   private final YoFrameVector desiredLinearCentroidalMomentumRate;
   private final YoFrameVector desiredAngularCentroidalMomentumRate;
   private final YoFrameVector linearCentroidalMomentumRateError;
   private final YoFrameVector angularCentroidalMomentumRateError;

   private final InverseDynamicsJoint[] jointsInOrder;


   public MomentumOptimizer(RigidBody elevator, ReferenceFrame centerOfMassFrame, double controlDT,
           YoVariableRegistry parentRegistry)
   {
      this.jointsInOrder = ScrewTools.computeJointsInOrder(elevator);
      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);

      jointVelocitiesMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
      jointAccelerationsMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
      m = Momentum.SIZE;
      n = 6;
      x = new double[n + 1];
      fvec = new double[m + 1];

      this.controlDT = controlDT;

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(), centroidalMomentumMatrix.getMatrix()
            .getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(), centroidalMomentumMatrix.getMatrix()
            .getNumCols());

      this.aQdd = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(), jointAccelerationsMatrix.getNumCols());
      this.adQd = new DenseMatrix64F(centroidalMomentumMatrixDerivative.getNumRows(), jointVelocitiesMatrix.getNumCols());

      this.desiredLinearCentroidalMomentumRate = new YoFrameVector("desiredLinearCentroidalMomentumRate", "", centerOfMassFrame, registry);
      this.desiredAngularCentroidalMomentumRate = new YoFrameVector("desiredAngularCentroidalMomentumRate", "", centerOfMassFrame, registry);

      linearCentroidalMomentumRateError = new YoFrameVector("linearCentroidalMomentumRateError", "", centerOfMassFrame, registry);
      angularCentroidalMomentumRateError = new YoFrameVector("angularCentroidalMomentumRateError", "", centerOfMassFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      centroidalMomentumMatrix.compute();
      previousCentroidalMomentumMatrix.set(centroidalMomentumMatrix.getMatrix());
   }

   public void solveForRootJointAcceleration(FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate)
   {
      this.desiredAngularCentroidalMomentumRate.set(desiredAngularCentroidalMomentumRate);
      this.desiredLinearCentroidalMomentumRate.set(desiredLinearCentroidalMomentumRate);

      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, jointVelocitiesMatrix);
      centroidalMomentumMatrix.compute();
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);

      updateBeforeSolving();

      info[1] = 0;
      Minpack_f77.lmdif1_f77(this, m, n, x, fvec, tol, info);    // also sets the result in the full robot model
      fcn(m, n, x, fvec, info);
   }

   public final void fcn(int m, int n, double[] x, double[] fvec, int[] iflag)
   {
      updateAtStartOfFcn(x);

      ScrewTools.packDesiredJointAccelerationsMatrix(jointsInOrder, jointAccelerationsMatrix);

      centroidalMomentumErrorMatrix.set(0, 0, desiredAngularCentroidalMomentumRate.getX());
      centroidalMomentumErrorMatrix.set(1, 0, desiredAngularCentroidalMomentumRate.getY());
      centroidalMomentumErrorMatrix.set(2, 0, desiredAngularCentroidalMomentumRate.getZ());
      centroidalMomentumErrorMatrix.set(3, 0, desiredLinearCentroidalMomentumRate.getX());
      centroidalMomentumErrorMatrix.set(4, 0, desiredLinearCentroidalMomentumRate.getY());
      centroidalMomentumErrorMatrix.set(5, 0, desiredLinearCentroidalMomentumRate.getZ());

      CommonOps.mult(centroidalMomentumMatrix.getMatrix(), jointAccelerationsMatrix, aQdd);
      CommonOps.mult(centroidalMomentumMatrixDerivative, jointVelocitiesMatrix, adQd);
      
      CommonOps.subEquals(centroidalMomentumErrorMatrix, aQdd);
      CommonOps.subEquals(centroidalMomentumErrorMatrix, adQd);
      

      angularCentroidalMomentumRateError.set(centroidalMomentumErrorMatrix.get(0, 0), centroidalMomentumErrorMatrix.get(1, 0),
            centroidalMomentumErrorMatrix.get(2, 0));
      linearCentroidalMomentumRateError.set(centroidalMomentumErrorMatrix.get(3, 0), centroidalMomentumErrorMatrix.get(4, 0),
            centroidalMomentumErrorMatrix.get(5, 0));

      int index = 1;
      for (int i = 0; i < centroidalMomentumErrorMatrix.getNumRows(); i++)
      {
         fvec[index++] = centroidalMomentumErrorMatrix.get(i, 0);
      }
   }

   protected abstract void updateBeforeSolving();
   
   protected abstract void updateAtStartOfFcn(double[] x);
}
