package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Vector3d;

import org.apache.commons.lang.ArrayUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

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
   private final GeometricJacobian jacobian;
   private final DenseMatrix64F selectionMatrix;
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();

   private final RigidBodyOrientationControlModule[] rigidBodyOrientationControlModules;
   private final RigidBody[] bases;
   private final IntegerYoVariable baseIndex;

   public DegenerateOrientationControlModule(String namePrefix, RigidBody[] bases, RigidBody endEffector, GeometricJacobian jacobian,
           TwistCalculator twistCalculator, YoVariableRegistry parentRegistry)
   {
      this.jacobian = jacobian;
      this.bases = bases;
      this.rigidBodyOrientationControlModules = new RigidBodyOrientationControlModule[bases.length];
      int i = 0;
      for (RigidBody base : bases)
      {
         String baseName = FormattingTools.capitalizeFirstLetter(base.getName());
         RigidBodyOrientationControlModule rigidBodyOrientationControlModule = new RigidBodyOrientationControlModule(namePrefix + baseName, base, endEffector,
                                                                                  twistCalculator, registry);
         rigidBodyOrientationControlModules[i++] = rigidBodyOrientationControlModule;
      }

      this.baseIndex = new IntegerYoVariable(namePrefix + "BaseIndex", registry);
      this.selectionMatrix = new DenseMatrix64F(jacobian.getNumberOfColumns(), Twist.SIZE);
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

      FrameVector angularAcceleration = new FrameVector(jacobian.getJacobianFrame());

      RigidBodyOrientationControlModule rigidBodyOrientationControlModule = rigidBodyOrientationControlModules[baseIndex.getIntegerValue()];
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

   public GeometricJacobian getJacobian()
   {
      return jacobian;
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

   public void setBase(RigidBody base)
   {
      int baseIndex = ArrayUtils.indexOf(bases, base);
      if (baseIndex == -1)
         throw new RuntimeException("Base not found: " + base.getName());
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
