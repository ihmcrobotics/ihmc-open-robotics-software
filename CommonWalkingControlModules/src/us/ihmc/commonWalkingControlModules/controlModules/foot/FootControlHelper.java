package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 5e-3;

   private final RobotSide robotSide;
   private final RigidBody rootBody;
   private final ContactableFoot contactableFoot;
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final long jacobianId;
   private final GeometricJacobian jacobian;
   private final EnumYoVariable<ConstraintType> requestedState;
   private final FrameVector fullyConstrainedNormalContactVector;
   private final BooleanYoVariable isDesiredCoPOnEdge;

   private final FrameConvexPolygon2d contactPolygon = new FrameConvexPolygon2d();

   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
   private final BooleanYoVariable doSingularityEscape;
   private final DoubleYoVariable singularityEscapeNullspaceMultiplier;
   private final DoubleYoVariable nullspaceMultiplier;
   private final DoubleYoVariable jacobianDeterminant;
   private final BooleanYoVariable jacobianDeterminantInRange;

   private final DoubleYoVariable minJacobianDeterminantForSingularityEscape;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final DenseMatrix64F selectionMatrix;

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
      double controlDT = momentumBasedController.getControlDT();

      partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, controlDT, contactableFoot, twistCalculator, walkingControllerParameters,
            registry, momentumBasedController.getDynamicGraphicObjectsListRegistry());

      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      RevoluteJoint[] legJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, foot), RevoluteJoint.class);
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(legJoints, foot.getBodyFixedFrame());
      jacobian = momentumBasedController.getJacobian(jacobianId);

      rootBody = twistCalculator.getRootBody();

      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);

      isDesiredCoPOnEdge = new BooleanYoVariable(namePrefix + "IsDesiredCoPOnEdge", registry);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      contactPolygon.setIncludingFrameAndUpdate(contactableFoot.getContactPoints2d());

      doSingularityEscape = new BooleanYoVariable(namePrefix + "DoSingularityEscape", registry);
      jacobianDeterminant = new DoubleYoVariable(namePrefix + "JacobianDeterminant", registry);
      jacobianDeterminantInRange = new BooleanYoVariable(namePrefix + "JacobianDeterminantInRange", registry);
      nullspaceMultiplier = new DoubleYoVariable(namePrefix + "NullspaceMultiplier", registry);
      singularityEscapeNullspaceMultiplier = new DoubleYoVariable(namePrefix + "SingularityEscapeNullspaceMultiplier", registry);

      minJacobianDeterminantForSingularityEscape = new DoubleYoVariable("minJacobianDeterminantForSingularityEscape", registry);
      minJacobianDeterminantForSingularityEscape.set(0.025);

      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
            walkingControllerParameters, momentumBasedController, registry);

      selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   private final FramePoint2d desiredCoP = new FramePoint2d();

   public void update()
   {
      momentumBasedController.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      if (desiredCoP.containsNaN())
         isDesiredCoPOnEdge.set(false);
      else
         isDesiredCoPOnEdge.set(!contactPolygon.isPointInside(desiredCoP, -EPSILON_POINT_ON_EDGE)); // Minus means that the check is done with a smaller polygon

      computeNullspaceMultipliers();
   }

   public boolean isCoPOnEdge()
   {
      return isDesiredCoPOnEdge.getBooleanValue();
   }

   public void computeNullspaceMultipliers()
   {
      jacobianDeterminant.set(jacobian.det());
      jacobianDeterminantInRange.set(Math.abs(jacobianDeterminant.getDoubleValue()) < minJacobianDeterminantForSingularityEscape.getDoubleValue());

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

   public void submitTaskspaceConstraint(SpatialAccelerationVector footAcceleration, SpatialAccelerationCommand spatialAccelerationCommandToPack)
   {
      submitTaskspaceConstraint(jacobianId, footAcceleration, spatialAccelerationCommandToPack);
   }

   public void submitTaskspaceConstraint(long jacobianId, SpatialAccelerationVector footAcceleration, SpatialAccelerationCommand spatialAccelerationCommandToPack)
   {
      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      spatialAccelerationCommandToPack.setJacobianForNullspaceId(jacobianId);
      spatialAccelerationCommandToPack.set(footAcceleration, nullspaceMultipliers, selectionMatrix);
      spatialAccelerationCommandToPack.set(rootBody, contactableFoot.getRigidBody());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ContactableFoot getContactableFoot()
   {
      return contactableFoot;
   }

   public MomentumBasedController getMomentumBasedController()
   {
      return momentumBasedController;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public PartialFootholdControlModule getPartialFootholdControlModule()
   {
      return partialFootholdControlModule;
   }

   public long getJacobianId()
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
