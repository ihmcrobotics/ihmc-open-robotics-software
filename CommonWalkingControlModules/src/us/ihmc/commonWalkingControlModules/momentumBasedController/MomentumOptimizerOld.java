package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.fed.fs.fpl.optimization.Lmdif_fcn;
import us.fed.fs.fpl.optimization.Minpack_f77;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public abstract class MomentumOptimizerOld implements Lmdif_fcn
{
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DenseMatrix64F jointVelocitiesMatrix;
   private final DenseMatrix64F jointAccelerationsMatrix;
   private final DenseMatrix64F centroidalMomentumErrorMatrix = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F avdot;
   private final DenseMatrix64F adotv;

   private final int m;
   private final int n;
   private final double[] fvec;
   private final double tol = 1e-9;
   private final int[] info = new int[2];

   protected final double controlDT;

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix; // to make numerical differentiation rewindable

   private final YoFrameVector desiredLinearCentroidalMomentumRate;
   private final YoFrameVector desiredAngularCentroidalMomentumRate;
   private final YoFrameVector linearCentroidalMomentumRateError;
   private final YoFrameVector angularCentroidalMomentumRateError;
   
   private final EnumYoVariable<LMDiffInfoCode> infoEnum = EnumYoVariable.create("lmdiffInfo", LMDiffInfoCode.class, registry);

   private final InverseDynamicsJoint[] jointsInOrder;


   public MomentumOptimizerOld(RigidBody elevator, ReferenceFrame centerOfMassFrame, double controlDT, YoVariableRegistry parentRegistry)
   {
//      this.jointsInOrder = ScrewTools.computeJointsInOrder(elevator); //deprecated method
      this.jointsInOrder = ScrewTools.computeSubtreeJoints(elevator); 
      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);

      jointVelocitiesMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
      jointAccelerationsMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
      m = Momentum.SIZE;
      n = 6;
      fvec = new double[m + 1];

      this.controlDT = controlDT;

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());

      this.avdot = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(), jointAccelerationsMatrix.getNumCols());
      this.adotv = new DenseMatrix64F(centroidalMomentumMatrixDerivative.getNumRows(), jointVelocitiesMatrix.getNumCols());

      this.desiredLinearCentroidalMomentumRate = new YoFrameVector("desiredLinearCentroidalMomentumRate", "", centerOfMassFrame, registry);
      this.desiredAngularCentroidalMomentumRate = new YoFrameVector("desiredAngularCentroidalMomentumRate", "", centerOfMassFrame, registry);

      linearCentroidalMomentumRateError = new YoFrameVector("linearCentroidalMomentumRateError", "", centerOfMassFrame, registry);
      angularCentroidalMomentumRateError = new YoFrameVector("angularCentroidalMomentumRateError", "", centerOfMassFrame, registry);

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);
      
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      centroidalMomentumMatrix.compute();
      previousCentroidalMomentumMatrix.set(centroidalMomentumMatrix.getMatrix());
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
   }

   public void solveForRootJointAcceleration(FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate)
   {
      double[] x = new double[n + 1];
      
      this.desiredAngularCentroidalMomentumRate.set(desiredAngularCentroidalMomentumRate);
      this.desiredLinearCentroidalMomentumRate.set(desiredLinearCentroidalMomentumRate);

      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, jointVelocitiesMatrix);
      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      updateBeforeSolving(x);

      info[1] = 0;
//      Minpack_f77.lmdif1_f77(this, m, n, initialGuess, fvec, tol, info);    // also sets the result in the full robot model


      // TODO: make fields:
      double diag[] = new double[n+1];
      int nfev[] = new int[2];
      double fjac[][] = new double[m+1][n+1];
      int ipvt[] = new int[n+1];
      double qtf[] = new double[n+1];


      double factor = 100.0;
      int maxfev = 200*(n + 1);
      double ftol = tol;
      double xtol = tol;
      double gtol = 0.0;
      double epsfcn = 0.0;
      int mode = 1;
      int nprint = 0;

      Minpack_f77.lmdif_f77(this, m, n, x, fvec, ftol, xtol, gtol, maxfev, epsfcn, diag, mode, factor, nprint, info, nfev, fjac, ipvt, qtf);
      
      infoEnum.set(LMDiffInfoCode.getCodeFromInt(info[1]));
      if (infoEnum.getEnumValue() != LMDiffInfoCode.SOL_ERROR_WITHIN_TOL && infoEnum.getEnumValue() != LMDiffInfoCode.SOS_AND_SOL_WITHIN_TOL)
         throw new RuntimeException("infoEnum = " + infoEnum.getEnumValue());
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

      CommonOps.mult(centroidalMomentumMatrix.getMatrix(), jointAccelerationsMatrix, avdot);
      CommonOps.mult(centroidalMomentumMatrixDerivative, jointVelocitiesMatrix, adotv);

      CommonOps.subtractEquals(centroidalMomentumErrorMatrix, avdot);
      CommonOps.subtractEquals(centroidalMomentumErrorMatrix, adotv);


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

   protected abstract void updateBeforeSolving(double[] x);

   protected abstract void updateAtStartOfFcn(double[] x);

   private static enum LMDiffInfoCode
   {
      IMPROPER_INPUT_PARAMETERS(0), SOS_ERROR_WITHIN_TOL(1), SOL_ERROR_WITHIN_TOL(2), SOS_AND_SOL_WITHIN_TOL(3), ORTHOGONAL_TO_JACOBIAN(4), MAX_CALLS_REACHED(5), TOL_TOO_SMALL_SOS(6), TOL_TOO_SMALL_SOL(7), TOL_TOO_SMALL_ORTH(8);
      private final int code;
      private LMDiffInfoCode(int code)
      {
         this.code = code;
      }

      private static LMDiffInfoCode getCodeFromInt(int code)
      {
         for (LMDiffInfoCode info : values())
            if (info.code == code)
               return info;
         throw new RuntimeException("Code " + code + " not recognized");
      }
   }
}
