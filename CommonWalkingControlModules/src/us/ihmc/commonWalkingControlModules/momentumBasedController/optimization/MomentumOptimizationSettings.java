package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXGenMomentumOptimizerBridge;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXGenMomentumOptimizerBridge.MomentumOptimizer;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * @author twan
 *         Date: 5/1/13
 */
public class MomentumOptimizationSettings
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable linearMomentumXYWeight = new DoubleYoVariable("linearMomentumXYWeight", registry);
   private final DoubleYoVariable linearMomentumZWeight = new DoubleYoVariable("linearMomentumZWeight", registry);
   private final DoubleYoVariable angularMomentumXYWeight = new DoubleYoVariable("angularMomentumXYWeight", registry);
   private final DoubleYoVariable angularMomentumZWeight = new DoubleYoVariable("angularMomentumZWeight", registry);
   private final DoubleYoVariable rhoMin = new DoubleYoVariable("rhoMin", registry);
   private final DoubleYoVariable lambda = new DoubleYoVariable("lambda", registry);
   private final InverseDynamicsJoint[] jointsToOptimizeFor;

   private final EnumYoVariable<MomentumOptimizer> activeMomentumOptimizer =
      new EnumYoVariable<CVXGenMomentumOptimizerBridge.MomentumOptimizer>("activeMomentumOptimizer", registry, MomentumOptimizer.class);

   private final double[] momentumWeightDiagonal = new double[Momentum.SIZE];
   private final DenseMatrix64F C = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private double wRhoCylinder;
   private double wPhiCylinder;
   private double wRhoPlane;
   private double wRhoSmoother;
   private double wRhoPenalizer;

   private final DenseMatrix64F momentumSubspaceProjector = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final Momentum tempMomentum = new Momentum();    // just to make sure that the ordering of force and torque are correct

   public MomentumOptimizationSettings(InverseDynamicsJoint[] jointsToOptimizeFor, YoVariableRegistry parentRegistry)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      parentRegistry.addChild(registry);
      
      activeMomentumOptimizer.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            if (activeMomentumOptimizer.getEnumValue() != MomentumOptimizer.GRF_PENALIZED_SMOOTHER)
            {
               System.err.println(getClass().getSimpleName() + ": Cannot switch to MomentumOptimizer: " + activeMomentumOptimizer.getEnumValue() + " until it is updated to consider 4 basis vectors.");
               activeMomentumOptimizer.set(MomentumOptimizer.GRF_PENALIZED_SMOOTHER);
            }
         }
      });
   }

   public void setMomentumWeight(double linearMomentumXYWeight, double linearMomentumZWeight, double angularMomentumXYWeight, double angularMomentumZWeight)
   {
      this.linearMomentumXYWeight.set(linearMomentumXYWeight);
      this.linearMomentumZWeight.set(linearMomentumZWeight);
      this.angularMomentumXYWeight.set(angularMomentumXYWeight);
      this.angularMomentumZWeight.set(angularMomentumZWeight);
   }

   public void setRhoCylinderContactRegularization(double wRho)
   {
      this.wRhoCylinder = wRho;
   }

   public void setPhiCylinderContactRegularization(double wPhi)
   {
      this.wPhiCylinder = wPhi;
   }

   public void setRhoPlaneContactRegularization(double wRho)
   {
      this.wRhoPlane = wRho;
   }

   public void setDampedLeastSquaresFactor(double lambda)
   {
      this.lambda.set(lambda);
   }

   public void setRhoMin(double rhoMin)
   {
      this.rhoMin.set(rhoMin);
   }

   public DenseMatrix64F getMomentumDotWeight(DenseMatrix64F momentumSubspace)
   {
      CommonOps.multOuter(momentumSubspace, momentumSubspaceProjector);

      tempMomentum.setLinearPartX(linearMomentumXYWeight.getDoubleValue());
      tempMomentum.setLinearPartY(linearMomentumXYWeight.getDoubleValue());
      tempMomentum.setLinearPartZ(linearMomentumZWeight.getDoubleValue());

      tempMomentum.setAngularPartX(angularMomentumXYWeight.getDoubleValue());
      tempMomentum.setAngularPartY(angularMomentumXYWeight.getDoubleValue());
      tempMomentum.setAngularPartZ(angularMomentumZWeight.getDoubleValue());

      tempMomentum.packMatrix(momentumWeightDiagonal);
      CommonOps.diag(tempMatrix, Momentum.SIZE, momentumWeightDiagonal);

      CommonOps.mult(momentumSubspaceProjector, tempMatrix, C);

      return C;
   }

   public double getRhoCylinderContactRegularization()
   {
      return wRhoCylinder;
   }

   public double getPhiCylinderContactRegularization()
   {
      return wPhiCylinder;
   }

   public double getRhoPlaneContactRegularization()
   {
      return wRhoPlane;
   }

   public void packDampedLeastSquaresFactorMatrix(DenseMatrix64F dampedLeastSquaresFactorMatrixToPack)
   {
      CommonOps.setIdentity(dampedLeastSquaresFactorMatrixToPack);
      CommonOps.scale(lambda.getDoubleValue(), dampedLeastSquaresFactorMatrixToPack);
   }

   public double getRhoMinScalar()
   {
      return rhoMin.getDoubleValue();
   }

   public double getDampedLeastSquaresFactor()
   {
      return lambda.getDoubleValue();
   }

   public double getRateOfChangeOfRhoPlaneContactRegularization()
   {
      return wRhoSmoother;
   }

   public double getPenalizerOfRhoPlaneContactRegularization()
   {
      return wRhoPenalizer;
   }

   public void setRhoPenalizerPlaneContactRegularization(double wRhoPenalizer)
   {
      this.wRhoPenalizer = wRhoPenalizer;
   }

   public void setRateOfChangeOfRhoPlaneContactRegularization(double wRhoSmoother)
   {
      this.wRhoSmoother = wRhoSmoother;
   }

   public void setMomentumOptimizerToUse(MomentumOptimizer momentumOptimizerToUse)
   {
      activeMomentumOptimizer.set(momentumOptimizerToUse);
   }

   public MomentumOptimizer getMomentumOptimizerToUse()
   {
      return activeMomentumOptimizer.getEnumValue();
   }

   public InverseDynamicsJoint[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }
}
