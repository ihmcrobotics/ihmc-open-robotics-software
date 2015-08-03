package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

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

   private final double[] momentumWeightDiagonal = new double[Momentum.SIZE];
   private final DenseMatrix64F C = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private double wRhoPlane;
   private double wRhoSmoother;
   private double wRhoPenalizer;

   private final DenseMatrix64F momentumSubspaceProjector = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final Momentum tempMomentum = new Momentum(); // just to make sure that the ordering of force and torque are correct

   // Parameters for shaky feet
   private boolean enableCoPSmootherControlForShakies = false;
   private double copErrorThresholdToTriggerSmoother = 0.10;
   private double copSmootherDuration = 0.5;
   private double maxWRhoSmoother = 1.5;

   public MomentumOptimizationSettings(InverseDynamicsJoint[] jointsToOptimizeFor, YoVariableRegistry parentRegistry)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      parentRegistry.addChild(registry);
   }

   public void setMomentumWeight(double linearMomentumXYWeight, double linearMomentumZWeight, double angularMomentumXYWeight, double angularMomentumZWeight)
   {
      this.linearMomentumXYWeight.set(linearMomentumXYWeight);
      this.linearMomentumZWeight.set(linearMomentumZWeight);
      this.angularMomentumXYWeight.set(angularMomentumXYWeight);
      this.angularMomentumZWeight.set(angularMomentumZWeight);
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

   public InverseDynamicsJoint[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public boolean getEnableCoPSmootherControlForShakies()
   {
      return enableCoPSmootherControlForShakies;
   }

   public void setEnableCoPSmootherControlForShakies(boolean enableCoPSmootherControlForShakies)
   {
      this.enableCoPSmootherControlForShakies = enableCoPSmootherControlForShakies;
   }

   public double getCopErrorThresholdToTriggerSmoother()
   {
      return copErrorThresholdToTriggerSmoother;
   }

   public void setCopErrorThresholdToTriggerSmoother(double copErrorThresholdToTriggerSmoother)
   {
      this.copErrorThresholdToTriggerSmoother = copErrorThresholdToTriggerSmoother;
   }

   public double getCoPSmootherDuration()
   {
      return copSmootherDuration;
   }

   public void setCopSmootherDuration(double copSmootherDuration)
   {
      this.copSmootherDuration = copSmootherDuration;
   }

   public double getMaxWRhoSmoother()
   {
      return maxWRhoSmoother;
   }

   public void setMaxWRhoSmoother(double maxWRhoSmoother)
   {
      this.maxWRhoSmoother = maxWRhoSmoother;
   }
}
