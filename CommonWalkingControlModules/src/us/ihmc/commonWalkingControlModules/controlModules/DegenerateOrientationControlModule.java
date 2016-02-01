package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;

import org.apache.commons.lang3.StringUtils;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.controllers.YoAxisAngleOrientationGains;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;


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

   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();

   private final String namePrefix;
   private final RigidBody endEffector;
   private final TwistCalculator twistCalculator;

   private final YoOrientationPIDGains gains;

   private final double controlDT;

   private final SelectionMatrixComputer selectionMatrixComputer = new SelectionMatrixComputer();
   
   public DegenerateOrientationControlModule(String namePrefix, RigidBody[] defaultBases, RigidBody endEffector, GeometricJacobian[] defaultJacobians,
         TwistCalculator twistCalculator, double controlDT, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, defaultBases, endEffector, defaultJacobians, twistCalculator, controlDT, null, parentRegistry);
   }

   public DegenerateOrientationControlModule(String namePrefix, RigidBody[] defaultBases, RigidBody endEffector, GeometricJacobian[] defaultJacobians,
         TwistCalculator twistCalculator, double controlDT, YoOrientationPIDGains gains, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.endEffector = endEffector;
      this.twistCalculator = twistCalculator;
      this.controlDT = controlDT;

      if (gains == null)
         gains = new YoAxisAngleOrientationGains(namePrefix, registry);

      this.gains = gains;

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

   public void reset()
   {
      for (int i = 0; i < rigidBodyOrientationControlModules.size(); i++)
      {
         rigidBodyOrientationControlModules.get(i).reset();
      }
   }

   public int addJacobian(GeometricJacobian jacobian)
   {
      jacobians.add(jacobian);
      this.selectionMatrices.add(new DenseMatrix64F(jacobian.getNumberOfColumns(), Twist.SIZE));

      int index = jacobians.size() - 1;
      if (index != selectionMatrices.size() - 1) throw new RuntimeException("RepInvariant Violation");

      jacobianIndex.set(index);

      return index;
   }

   public int addBase(RigidBody base)
   {
      bases.add(base);
      String baseName = StringUtils.capitalize(base.getName());
      RigidBodyOrientationControlModule rigidBodyOrientationControlModule = new RigidBodyOrientationControlModule(namePrefix + baseName, base, endEffector,
            twistCalculator, controlDT, gains, registry);

      rigidBodyOrientationControlModules.add(rigidBodyOrientationControlModule);

      int index = bases.size() - 1;
      if (index != rigidBodyOrientationControlModules.size() - 1)
         throw new RuntimeException("RepInvariant Violation");

      baseIndex.set(index);

      return index;
   }

   protected abstract void packDesiredAngularAccelerationFeedForward(FrameVector angularAccelerationToPack);

   protected abstract void packDesiredAngularVelocity(FrameVector angularVelocityToPack);

   protected abstract void packDesiredFrameOrientation(FrameOrientation orientationToPack);

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector zeroLinearAcceleration = new FrameVector();
   private final FrameVector controlledAngularAcceleration = new FrameVector();

   public void compute()
   {
      if (jacobians.get(jacobianIndex.getIntegerValue()).getNumberOfColumns() == 0) return;

      packDesiredFrameOrientation(desiredOrientation);
      packDesiredAngularVelocity(desiredAngularVelocity);
      packDesiredAngularAccelerationFeedForward(feedForwardAngularAcceleration);

      int localJacobianIndex = this.jacobianIndex.getIntegerValue();
      if (localJacobianIndex == -1) return;

      GeometricJacobian jacobian = jacobians.get(localJacobianIndex);
      DenseMatrix64F selectionMatrix = selectionMatrices.get(localJacobianIndex);

      ReferenceFrame expressedInFrame = jacobian.getJacobianFrame();
      controlledAngularAcceleration.setToZero(expressedInFrame);

      int localBaseIndex = baseIndex.getIntegerValue();
      if (localBaseIndex == -1) return;

      RigidBodyOrientationControlModule rigidBodyOrientationControlModule = rigidBodyOrientationControlModules.get(localBaseIndex);
      rigidBodyOrientationControlModule.compute(controlledAngularAcceleration, desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      ReferenceFrame endEffectorFrame = rigidBodyOrientationControlModule.getEndEffector().getBodyFixedFrame();
      ReferenceFrame baseFrame = rigidBodyOrientationControlModule.getBase().getBodyFixedFrame();
      zeroLinearAcceleration.setToZero(expressedInFrame);
      spatialAcceleration.set(endEffectorFrame, baseFrame, expressedInFrame, zeroLinearAcceleration, controlledAngularAcceleration);

      selectionMatrixComputer.computeSelectionMatrix(jacobian, selectionMatrix);
   }

   public TaskspaceConstraintData getTaskspaceConstraintData()
   {
      DenseMatrix64F selectionMatrix = selectionMatrices.get(jacobianIndex.getIntegerValue());

      taskspaceConstraintData.set(bases.get(baseIndex.getIntegerValue()), endEffector);
      taskspaceConstraintData.set(spatialAcceleration, nullspaceMultipliers, selectionMatrix);
      return taskspaceConstraintData;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      gains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      gains.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      gains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
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
      if (localJacobianIndex == -1)
         return null;

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

   
}
