package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Vector3d;

import org.apache.commons.lang.ArrayUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public abstract class DegenerateOrientationControlModule
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GeometricJacobian[] jacobians;
   private final DenseMatrix64F[] selectionMatrices;
   private final IntegerYoVariable jacobianIndex;

   private final RigidBodyOrientationControlModule[] rigidBodyOrientationControlModules;
   private final RigidBody[] bases;
   private final IntegerYoVariable baseIndex;
   
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();

   public DegenerateOrientationControlModule(String namePrefix, RigidBody[] bases, RigidBody endEffector, GeometricJacobian[] jacobians,
           TwistCalculator twistCalculator, YoVariableRegistry parentRegistry)
   {
      this.jacobians = jacobians;
      this.selectionMatrices = new DenseMatrix64F[jacobians.length];
      for (int i=0; i<jacobians.length; i++)
      {
         GeometricJacobian jacobian = jacobians[i];
         this.selectionMatrices[i] = new DenseMatrix64F(jacobian.getNumberOfColumns(), Twist.SIZE);
      }
      this.jacobianIndex = new IntegerYoVariable(namePrefix + "JacobianIndex", registry);
      jacobianIndex.set(0);
      

      this.bases = bases;
      this.rigidBodyOrientationControlModules = new RigidBodyOrientationControlModule[bases.length];
      for (int i=0; i<bases.length; i++)
      {
         RigidBody base = bases[i];
         String baseName = FormattingTools.capitalizeFirstLetter(base.getName());
         RigidBodyOrientationControlModule rigidBodyOrientationControlModule = new RigidBodyOrientationControlModule(namePrefix + baseName, base, endEffector,
                                                                                  twistCalculator, registry);
         rigidBodyOrientationControlModules[i] = rigidBodyOrientationControlModule;
      }

      this.baseIndex = new IntegerYoVariable(namePrefix + "BaseIndex", registry);
      this.baseIndex.set(0);
      
      parentRegistry.addChild(registry);
   }

   protected abstract FrameVector getDesiredAngularAccelerationFeedForward();

   protected abstract FrameVector getDesiredAngularVelocity();

   protected abstract FrameOrientation getDesiredFrameOrientation();

   public void compute()
   {
      FrameOrientation desiredOrientation = getDesiredFrameOrientation();
      FrameVector desiredAngularVelocity = getDesiredAngularVelocity();
      FrameVector feedForwardAngularAcceleration = getDesiredAngularAccelerationFeedForward();

      int localJacobianIndex = this.jacobianIndex.getIntegerValue();
      GeometricJacobian jacobian = jacobians[localJacobianIndex];
      DenseMatrix64F selectionMatrix = selectionMatrices[localJacobianIndex];

      FrameVector angularAcceleration = new FrameVector(jacobian .getJacobianFrame());

      RigidBodyOrientationControlModule rigidBodyOrientationControlModule = rigidBodyOrientationControlModules[baseIndex.getIntegerValue()];
      rigidBodyOrientationControlModule.compute(angularAcceleration, desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      spatialAcceleration.set(rigidBodyOrientationControlModule.getEndEffector().getBodyFixedFrame(),
                              rigidBodyOrientationControlModule.getBase().getBodyFixedFrame(), jacobian.getJacobianFrame(), new Vector3d(),
                              angularAcceleration.getVector());

      computeSelectionMatrix(jacobian, selectionMatrix);
   }

   public TaskspaceConstraintData getTaskspaceConstraintData()
   {
      DenseMatrix64F selectionMatrix = selectionMatrices[jacobianIndex.getIntegerValue()];

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(spatialAcceleration, nullspaceMultipliers, selectionMatrix);
      return taskspaceConstraintData;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      for (RigidBodyOrientationControlModule rigidBodyOrientationControlModule : rigidBodyOrientationControlModules)
      {
         rigidBodyOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      }
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      for (RigidBodyOrientationControlModule rigidBodyOrientationControlModule : rigidBodyOrientationControlModules)
      {
         rigidBodyOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
      }
   }

   public void setJacobian(GeometricJacobian jacobian)
   {
      int jacobianIndex = ArrayUtils.indexOf(jacobians, jacobian);
      if (jacobianIndex == -1)
         throw new RuntimeException("Jacobian not found: " + jacobian);
      
      setJacobian(jacobianIndex);
   }
   
   public void setJacobian(int jacobianIndex)
   {
      this.jacobianIndex.set(jacobianIndex);
   }
   
   public GeometricJacobian[] getAvailableJacobians()
   {
      return jacobians;
   }
   
   public GeometricJacobian getJacobian()
   {
      return jacobians[jacobianIndex.getIntegerValue()];
   }
   
   public void setBase(RigidBody base)
   {
      int baseIndex = ArrayUtils.indexOf(bases, base);
      if (baseIndex == -1)
         throw new RuntimeException("Base not found: " + base.getName());
      
      setBase(baseIndex);
   }  
   
   public void setBase(int baseIndex)
   {
      this.baseIndex.set(baseIndex);
   }

   public RigidBody[] getAvailableBases()
   {
      return bases;
   }

   private static void computeSelectionMatrix(GeometricJacobian jacobian, DenseMatrix64F selectionMatrix)
   {
      jacobian.compute();
      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      CommonOps.pinv(jacobianMatrix, selectionMatrix);
   }


}
