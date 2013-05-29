package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

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
   private final ArrayList<GeometricJacobian> jacobians = new ArrayList<GeometricJacobian>();
   private final ArrayList<DenseMatrix64F> selectionMatrices = new ArrayList<DenseMatrix64F>();
   private final IntegerYoVariable jacobianIndex;

   private final ArrayList<RigidBodyOrientationControlModule> rigidBodyOrientationControlModules = new ArrayList<RigidBodyOrientationControlModule>();
   private final ArrayList<RigidBody> bases = new ArrayList<RigidBody>();
   private final IntegerYoVariable baseIndex;
   
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();

   private final String namePrefix;
   private final RigidBody endEffector;
   private final TwistCalculator twistCalculator;
   
   public DegenerateOrientationControlModule(String namePrefix, RigidBody[] defaultBases, RigidBody endEffector, GeometricJacobian[] defaultJacobians,
           TwistCalculator twistCalculator, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.endEffector = endEffector;
      this.twistCalculator = twistCalculator;
      
      this.jacobianIndex = new IntegerYoVariable(namePrefix + "JacobianIndex", registry);
      jacobianIndex.set(-1);

      for (GeometricJacobian jacobian : defaultJacobians)
      {
         addJacobian(jacobian);
      }
      
      this.baseIndex = new IntegerYoVariable(namePrefix + "BaseIndex", registry);
      this.baseIndex.set(-1);

      for (RigidBody base : defaultBases)
      {
         addBase(base);
      }

      
      parentRegistry.addChild(registry);
   }
   
   public int addJacobian(GeometricJacobian jacobian)
   {      
      jacobians.add(jacobian);
      this.selectionMatrices.add(new DenseMatrix64F(jacobian.getNumberOfColumns(), Twist.SIZE));
      
      int index = jacobians.size()-1;
      if (index != selectionMatrices.size() - 1) throw new RuntimeException("RepInvariant Violation");
      
      jacobianIndex.set(index);
      
      return index;
   }
   
   public int addBase(RigidBody base)
   {
      bases.add(base);
      String baseName = FormattingTools.capitalizeFirstLetter(base.getName());
      RigidBodyOrientationControlModule rigidBodyOrientationControlModule = new RigidBodyOrientationControlModule(namePrefix + baseName, base, endEffector,
                                                                               twistCalculator, registry);
      rigidBodyOrientationControlModules.add(rigidBodyOrientationControlModule);
      
      int index = bases.size()-1;
      if (index != rigidBodyOrientationControlModules.size() - 1) throw new RuntimeException("RepInvariant Violation");
      
      baseIndex.set(index);
      
      return index;
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
      if (localJacobianIndex == -1) return;
      
      GeometricJacobian jacobian = jacobians.get(localJacobianIndex);
      DenseMatrix64F selectionMatrix = selectionMatrices.get(localJacobianIndex);

      FrameVector angularAcceleration = new FrameVector(jacobian.getJacobianFrame());

      int localBaseIndex = baseIndex.getIntegerValue();
      if (localBaseIndex == -1) return;

      RigidBodyOrientationControlModule rigidBodyOrientationControlModule = rigidBodyOrientationControlModules.get(localBaseIndex);
      rigidBodyOrientationControlModule.compute(angularAcceleration, desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      spatialAcceleration.set(rigidBodyOrientationControlModule.getEndEffector().getBodyFixedFrame(),
                              rigidBodyOrientationControlModule.getBase().getBodyFixedFrame(), jacobian.getJacobianFrame(), new Vector3d(),
                              angularAcceleration.getVector());

      computeSelectionMatrix(jacobian, selectionMatrix);
   }

   public TaskspaceConstraintData getTaskspaceConstraintData()
   {
      DenseMatrix64F selectionMatrix = selectionMatrices.get(jacobianIndex.getIntegerValue());

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
      if (jacobian == null)
      {
         this.jacobianIndex.set(-1);
         return;
      }
      
      int jacobianIndex = jacobians.indexOf(jacobian);
      if (jacobianIndex == -1)
      {
         jacobianIndex = addJacobian(jacobian);
      }
      
      setJacobian(jacobianIndex);
   }
   
   public void setJacobian(int jacobianIndex)
   {
      this.jacobianIndex.set(jacobianIndex);
   }
   
   public ArrayList<GeometricJacobian> getAvailableJacobians()
   {
      return jacobians;
   }
   
   public GeometricJacobian getJacobian()
   {
      int localJacobianIndex = jacobianIndex.getIntegerValue();
      if (localJacobianIndex == -1) return null;
      
      return jacobians.get(localJacobianIndex);
   }
   
   public void setBase(RigidBody base)
   {
      if (base == null)
      {
         this.baseIndex.set(-1);
         return;
      }
      
      int baseIndex = bases.indexOf(base);
      if (baseIndex == -1)
      {
         baseIndex = addBase(base);
      }

      setBase(baseIndex);
   }  
   
   public void setBase(int baseIndex)
   {
      this.baseIndex.set(baseIndex);
   }

   public ArrayList<RigidBody> getAvailableBases()
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
