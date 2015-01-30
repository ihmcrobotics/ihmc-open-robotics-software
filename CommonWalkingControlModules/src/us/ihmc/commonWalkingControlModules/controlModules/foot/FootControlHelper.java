package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 1e-2;
   private static final double minJacobianDeterminant = 0.035;

   private final RobotSide robotSide;
   private final RigidBody rootBody;
   private final ContactablePlaneBody contactableFoot;
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;
   private final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final int jacobianId;
   private final GeometricJacobian jacobian;
   private final EnumYoVariable<ConstraintType> requestedState;
   private final FrameVector fullyConstrainedNormalContactVector;

   private final FrameConvexPolygon2d contactPolygon = new FrameConvexPolygon2d();

   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
   private final BooleanYoVariable doSingularityEscape;
   private final DoubleYoVariable singularityEscapeNullspaceMultiplier;
   private final DoubleYoVariable nullspaceMultiplier;
   private final DoubleYoVariable jacobianDeterminant;
   private final BooleanYoVariable jacobianDeterminantInRange;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final DenseMatrix64F selectionMatrix;
   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();

   public FootControlHelper(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, MomentumBasedController momentumBasedController,
         YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.momentumBasedController = momentumBasedController;
      this.walkingControllerParameters = walkingControllerParameters;

      contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      twistCalculator = momentumBasedController.getTwistCalculator();

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();
      ReferenceFrame frameAfterAnkle = contactableFoot.getFrameAfterParentJoint();
      double controlDT = momentumBasedController.getControlDT();

      accelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, foot, frameAfterAnkle, controlDT, registry);

      partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, controlDT, contactableFoot, twistCalculator, walkingControllerParameters,
            registry, momentumBasedController.getDynamicGraphicObjectsListRegistry());


      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
      jacobian = momentumBasedController.getJacobian(jacobianId);

      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      contactPolygon.setIncludingFrameAndUpdate(contactableFoot.getContactPoints2d());

      doSingularityEscape = new BooleanYoVariable(namePrefix + "DoSingularityEscape", registry);
      jacobianDeterminant = new DoubleYoVariable(namePrefix + "JacobianDeterminant", registry);
      jacobianDeterminantInRange = new BooleanYoVariable(namePrefix + "JacobianDeterminantInRange", registry);
      nullspaceMultiplier = new DoubleYoVariable(namePrefix + "NullspaceMultiplier", registry);
      singularityEscapeNullspaceMultiplier = new DoubleYoVariable(namePrefix + "SingularityEscapeNullspaceMultiplier", registry);

      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
            walkingControllerParameters, momentumBasedController, registry);

      selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      rootBody = twistCalculator.getRootBody();
      taskspaceConstraintData.set(rootBody, contactableFoot.getRigidBody());
   }

   public boolean isCoPOnEdge()
   {
      FramePoint2d cop = momentumBasedController.getDesiredCoP(contactableFoot);

      if (cop == null || cop.containsNaN())
         return false;
      else
         return !contactPolygon.isPointInside(cop, EPSILON_POINT_ON_EDGE);
   }

   public void computeNullspaceMultipliers()
   {
      jacobianDeterminant.set(jacobian.det());
      jacobianDeterminantInRange.set(Math.abs(jacobianDeterminant.getDoubleValue()) < minJacobianDeterminant);

      if (jacobianDeterminantInRange.getBooleanValue())
      {
         nullspaceMultipliers.reshape(1, 1);
         if (doSingularityEscape.getBooleanValue())
         {
            nullspaceMultipliers.set(0, nullspaceMultiplier.getDoubleValue());
         }
         else
         {
            nullspaceMultipliers.set(0, 0);
         }
      }
      else
      {
         nullspaceMultiplier.set(Double.NaN);
         nullspaceMultipliers.reshape(0, 1);
         doSingularityEscape.set(false);
      }
   }

   public void submitTaskspaceConstraint(SpatialAccelerationVector footAcceleration)
   {
      submitTaskspaceConstraint(jacobianId, footAcceleration);
   }

   public void submitTaskspaceConstraint(int jacobianId, SpatialAccelerationVector footAcceleration)
   {
      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      taskspaceConstraintData.set(footAcceleration, nullspaceMultipliers, selectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ContactablePlaneBody getContactableFoot()
   {
      return contactableFoot;
   }

   public MomentumBasedController getMomentumBasedController()
   {
      return momentumBasedController;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public RigidBodySpatialAccelerationControlModule getAccelerationControlModule()
   {
      return accelerationControlModule;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public PartialFootholdControlModule getPartialFootholdControlModule()
   {
      return partialFootholdControlModule;
   }

   public int getJacobianId()
   {
      return jacobianId;
   }

   public GeometricJacobian getJacobian()
   {
      return jacobian;
   }

   public void requestState(ConstraintType requestedState)
   {
      this.requestedState.set(requestedState);
   }

   public ConstraintType getRequestedState()
   {
      return requestedState.getEnumValue();
   }

   public void setRequestedStateAsProcessed()
   {
      requestedState.set(null);
   }

   public void setFullyConstrainedNormalContactVector(FrameVector normalContactVector)
   {
      if (normalContactVector != null)
         fullyConstrainedNormalContactVector.setIncludingFrame(normalContactVector);
      else
         fullyConstrainedNormalContactVector.setIncludingFrame(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
   }

   public FrameVector getFullyConstrainedNormalContactVector()
   {
      return fullyConstrainedNormalContactVector;
   }

   public DenseMatrix64F getNullspaceMultipliers()
   {
      return nullspaceMultipliers;
   }

   public void resetNullspaceMultipliers()
   {
      nullspaceMultipliers.reshape(0, 1);
   }

   public boolean isDoingSingularityEscape()
   {
      return doSingularityEscape.getBooleanValue();
   }

   public void resetSingularityEscape()
   {
      doSingularityEscape.set(false);
   }

   public void doSingularityEscape(boolean doSingularityEscape)
   {
      this.doSingularityEscape.set(doSingularityEscape);
      this.nullspaceMultiplier.set(singularityEscapeNullspaceMultiplier.getDoubleValue());
   }

   public void doSingularityEscape(double temporarySingularityEscapeNullspaceMultiplier)
   {
      doSingularityEscape.set(true);
      this.nullspaceMultiplier.set(temporarySingularityEscapeNullspaceMultiplier);
   }

   public void setNullspaceMultiplier(double singularityEscapeNullspaceMultiplier)
   {
      this.singularityEscapeNullspaceMultiplier.set(singularityEscapeNullspaceMultiplier);
   }

   public double getJacobianDeterminant()
   {
      return jacobianDeterminant.getDoubleValue();
   }

   public boolean isJacobianDeterminantInRange()
   {
      return jacobianDeterminantInRange.getBooleanValue();
   }

   public LegSingularityAndKneeCollapseAvoidanceControlModule getLegSingularityAndKneeCollapseAvoidanceControlModule()
   {
      return legSingularityAndKneeCollapseAvoidanceControlModule;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void resetSelectionMatrix()
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }
}
