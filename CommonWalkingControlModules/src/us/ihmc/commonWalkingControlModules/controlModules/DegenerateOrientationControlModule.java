package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public abstract class DegenerateOrientationControlModule
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GeometricJacobian jacobian;
   private final DenseMatrix64F selectionMatrix;
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();

   private final RigidBodyOrientationControlModule rigidBodyOrientationControlModule;

   public DegenerateOrientationControlModule(String namePrefix, RigidBody base, RigidBody endEffector, GeometricJacobian jacobian,
           TwistCalculator twistCalculator)
   {
      this.jacobian = jacobian;
      this.rigidBodyOrientationControlModule = new RigidBodyOrientationControlModule(namePrefix, base, endEffector, twistCalculator, registry);
      this.selectionMatrix = new DenseMatrix64F(jacobian.getNumberOfColumns(), Twist.SIZE);
   }

   protected abstract FrameVector getDesiredAngularAccelerationFeedForward();

   protected abstract FrameVector getDesiredAngularVelocity();

   protected abstract FrameOrientation getDesiredFrameOrientation();

   public void compute()
   {
      FrameOrientation desiredOrientation = getDesiredFrameOrientation();
      FrameVector desiredAngularVelocity = getDesiredAngularVelocity();
      FrameVector feedForwardAngularAcceleration = getDesiredAngularAccelerationFeedForward();

      FrameVector angularAcceleration = new FrameVector(jacobian.getJacobianFrame());
      rigidBodyOrientationControlModule.compute(angularAcceleration, desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      spatialAcceleration.set(rigidBodyOrientationControlModule.getEndEffector().getBodyFixedFrame(),
                              rigidBodyOrientationControlModule.getBase().getBodyFixedFrame(), jacobian.getJacobianFrame(), new Vector3d(),
                              angularAcceleration.getVector());

      computeSelectionMatrix(jacobian, selectionMatrix);
   }

   public SpatialAccelerationVector getSpatialAcceleration()
   {
      return spatialAcceleration;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      rigidBodyOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      rigidBodyOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   private static void computeSelectionMatrix(GeometricJacobian jacobian, DenseMatrix64F selectionMatrix)
   {
      jacobian.compute();
      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      CommonOps.pinv(jacobianMatrix, selectionMatrix);
   }
}
