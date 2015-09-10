package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.List;

import javax.vecmath.Point2d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameMatrix3D;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 5e-3;

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
   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();

   private final BooleanYoVariable isFootRollUncontrollable;

   private final FrameVector hipYawAxis = new FrameVector();
   private final FrameVector ankleRollAxis = new FrameVector();

   private final DoubleYoVariable ankleRollAndHipYawAlignmentFactor;
   private final DoubleYoVariable ankleRollAndHipYawAlignmentTreshold;
   private final FrameMatrix3D angularSelectionMatrix = new FrameMatrix3D();

   // For hold position and fully constrained states.
   private final FrameTuple2dArrayList<FramePoint2d> defaultContactPointPositions = FrameTuple2dArrayList.createFramePoint2dArrayList(4);
   private final FrameTuple2dArrayList<FramePoint2d> originalContactPointPositions = FrameTuple2dArrayList.createFramePoint2dArrayList(4);
   private final FrameConvexPolygon2d originalSupportFootPolygon = new FrameConvexPolygon2d();
   private final BooleanYoVariable allowSupportFootShrink;
   private final BooleanYoVariable isSupportFootShrinkEnabled;
   private final DoubleYoVariable contactPointMaxX;
   private final DoubleYoVariable alphaForFootShrink;
   private final AlphaFilteredYoVariable footShrinkPercent;
   private final DoubleYoVariable maxFootShrinkInPercent;
   private final DoubleYoVariable thresholdForAnkleLimitWatcher;
   private final BooleanYoVariable isAnkleFlexionLimitReached;

   public FootControlHelper(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, MomentumBasedController momentumBasedController,
         YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.momentumBasedController = momentumBasedController;
      this.walkingControllerParameters = walkingControllerParameters;

      contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      twistCalculator = momentumBasedController.getTwistCalculator();

      defaultContactPointPositions.copyFromListAndTrimSize(contactableFoot.getContactPoints2d());

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();
      ReferenceFrame frameAfterAnkle = contactableFoot.getFrameAfterParentJoint();
      double controlDT = momentumBasedController.getControlDT();

      accelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, foot, frameAfterAnkle, controlDT, registry);

      partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, controlDT, contactableFoot, twistCalculator, walkingControllerParameters,
            registry, momentumBasedController.getDynamicGraphicObjectsListRegistry());


      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      RevoluteJoint[] legJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, foot), RevoluteJoint.class);
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(legJoints, foot.getBodyFixedFrame());
      jacobian = momentumBasedController.getJacobian(jacobianId);

      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);

      isDesiredCoPOnEdge = new BooleanYoVariable(namePrefix + "IsDesiredCoPOnEdge", registry);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      contactPolygon.setIncludingFrameAndUpdate(contactableFoot.getContactPoints2d());

      doSingularityEscape = new BooleanYoVariable(namePrefix + "DoSingularityEscape", registry);
      jacobianDeterminant = new DoubleYoVariable(namePrefix + "JacobianDeterminant", registry);
      jacobianDeterminantInRange = new BooleanYoVariable(namePrefix + "JacobianDeterminantInRange", registry);
      nullspaceMultiplier = new DoubleYoVariable(namePrefix + "NullspaceMultiplier", registry);
      singularityEscapeNullspaceMultiplier = new DoubleYoVariable(namePrefix + "SingularityEscapeNullspaceMultiplier", registry);
      isFootRollUncontrollable = new BooleanYoVariable(namePrefix + "IsFootRollUncontrollable", registry);
      ankleRollAndHipYawAlignmentFactor = new DoubleYoVariable(namePrefix + "AkleRollAndHipYawAlignmentFactor", registry);
      ankleRollAndHipYawAlignmentTreshold = new DoubleYoVariable(namePrefix + "AkleRollAndHipYawAlignmentThreshold", registry);
      ankleRollAndHipYawAlignmentTreshold.set(0.9);

      minJacobianDeterminantForSingularityEscape = new DoubleYoVariable("minJacobianDeterminantForSingularityEscape", registry);
      minJacobianDeterminantForSingularityEscape.set(0.025);
      
      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
            walkingControllerParameters, momentumBasedController, registry);

      selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      rootBody = twistCalculator.getRootBody();
      taskspaceConstraintData.set(rootBody, contactableFoot.getRigidBody());

      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      isSupportFootShrinkEnabled = new BooleanYoVariable("is" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SupportFootShrinkEnabled", registry);
      allowSupportFootShrink = new BooleanYoVariable("AllowSupport" + robotSide.getCamelCaseNameForMiddleOfExpression() + "FootShrink", registry);
      allowSupportFootShrink.set(walkingControllerParameters.allowShrinkingSingleSupportFootPolygon());
      contactPointMaxX = new DoubleYoVariable(contactableFoot.getName() + "ContactPointMaxX", registry);
      contactPointMaxX.set(Double.NEGATIVE_INFINITY);
      alphaForFootShrink = new DoubleYoVariable(sidePrefix + "AlphaForFootShrink", registry);
      alphaForFootShrink.set(0.95);
      footShrinkPercent = new AlphaFilteredYoVariable(sidePrefix + "FootShrinkPercent", registry, alphaForFootShrink);
      maxFootShrinkInPercent = new DoubleYoVariable(sidePrefix + "MaxFootShrinkInPercent", registry);
      maxFootShrinkInPercent.set(1.0);
      thresholdForAnkleLimitWatcher = new DoubleYoVariable(sidePrefix + "ThresholdInPercentForAnkleLimitWatcher", registry);
      thresholdForAnkleLimitWatcher.set(0.08);
      isAnkleFlexionLimitReached = new BooleanYoVariable(sidePrefix + "IsAnkleLimitReached", registry);
   }

   public void update()
   {
      FramePoint2d cop = momentumBasedController.getDesiredCoP(contactableFoot);

      if (cop == null || cop.containsNaN())
         isDesiredCoPOnEdge.set(false);
      else
         isDesiredCoPOnEdge.set(!contactPolygon.isPointInside(cop, -EPSILON_POINT_ON_EDGE)); // Minus means that the check is done with a smaller polygon
   
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
      computeJointsAlignmentFactor();

      if (jacobianDeterminantInRange.getBooleanValue() && ankleRollAndHipYawAlignmentFactor.getDoubleValue() < ankleRollAndHipYawAlignmentTreshold.getDoubleValue())
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

   private void computeJointsAlignmentFactor()
   {
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW).getJointAxis(hipYawAxis);
      fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL).getJointAxis(ankleRollAxis);

      ankleRollAxis.changeFrame(hipYawAxis.getReferenceFrame());
      ankleRollAndHipYawAlignmentFactor.set(Math.abs(ankleRollAxis.dot(hipYawAxis)));
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

   public void updateSelectionMatrixToHandleAnkleRollAndHipYawAlignment()
   {
      if (ankleRollAndHipYawAlignmentFactor.getDoubleValue() > ankleRollAndHipYawAlignmentTreshold.getDoubleValue())
      {
         isFootRollUncontrollable.set(true);
         ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
         ReferenceFrame jacobianFrame = jacobian.getJacobianFrame();
         angularSelectionMatrix.setToIdentity(footFrame);
         double s22 = 10.0 * (1.0 - ankleRollAndHipYawAlignmentFactor.getDoubleValue());
         angularSelectionMatrix.setM22(s22);
         angularSelectionMatrix.changeFrame(jacobianFrame);
         angularSelectionMatrix.getDenseMatrix(selectionMatrix, 0, 0);
         OneDoFJoint ankleRollJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_ROLL);
         momentumBasedController.doPDControl(ankleRollJoint, 1.0, 0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }
      else
      {
         isFootRollUncontrollable.set(false);
         selectionMatrix.set(0, 0, 1.0);
         selectionMatrix.set(1, 1, 1.0);
         selectionMatrix.set(2, 2, 1.0);
      }
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

   public RigidBodySpatialAccelerationControlModule getAccelerationControlModule()
   {
      return accelerationControlModule;
   }

   public void setGains(YoSE3PIDGains gains)
   {
      accelerationControlModule.setGains(gains);
   }

   public void setOrientationGains(YoOrientationPIDGains gains)
   {
      accelerationControlModule.setOrientationGains(gains);
   }

   public void setGainsToZero()
   {
      accelerationControlModule.setPositionProportionalGains(0.0, 0.0, 0.0);
      accelerationControlModule.setPositionDerivativeGains(0.0, 0.0, 0.0);
      accelerationControlModule.setPositionIntegralGains(0.0, 0.0, 0.0, 0.0);
      accelerationControlModule.setPositionMaxAccelerationAndJerk(0.0, 0.0);
      accelerationControlModule.setOrientationProportionalGains(0.0, 0.0, 0.0);
      accelerationControlModule.setOrientationDerivativeGains(0.0, 0.0, 0.0);
      accelerationControlModule.setOrientationIntegralGains(0.0, 0.0, 0.0, 0.0);
      accelerationControlModule.setOrientationMaxAccelerationAndJerk(0.0, 0.0);
   }

   public void resetAccelerationControlModule()
   {
      accelerationControlModule.reset();
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

   public void enableAnkleLimitAvoidanceInSupportState(boolean enable)
   {
      if (!allowSupportFootShrink.getBooleanValue())
         enable = false;

      isSupportFootShrinkEnabled.set(enable);

      if (!enable)
         footShrinkPercent.set(0.0);
   }

   public void saveContactPointsFromContactState()
   {
      momentumBasedController.getContactState(contactableFoot).getAllContactPoints(originalContactPointPositions);
   }

   public void saveContactPointsFromFootstep(Footstep footstep)
   {
      List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints == null || predictedContactPoints.isEmpty())
         originalContactPointPositions.copyFromListAndTrimSize(defaultContactPointPositions);
      else
         originalContactPointPositions.copyFromPoint2dListAndTrimSize(contactableFoot.getSoleFrame(), predictedContactPoints);
   }

   public void initializeParametersForSupportFootShrink(boolean resetCurrentFootShrink)
   {
      saveContactPointsFromContactState();
      updateOriginalSupportPolygon(resetCurrentFootShrink);
   }

   public void updateWithPredictedContactPoints(Footstep footstep)
   {
      saveContactPointsFromFootstep(footstep);
      updateOriginalSupportPolygon(false);
   }

   private void updateOriginalSupportPolygon(boolean resetCurrentFootShrink)
   {
      contactPointMaxX.set(Double.NEGATIVE_INFINITY);
      for (int i = 0; i < originalContactPointPositions.size(); i++)
      {
         if (originalContactPointPositions.get(i).getX() > contactPointMaxX.getDoubleValue())
            contactPointMaxX.set(originalContactPointPositions.get(i).getX());
      }

      originalSupportFootPolygon.setIncludingFrameAndUpdate(originalContactPointPositions);

      if (resetCurrentFootShrink)
      {
         footShrinkPercent.set(0.0);
      }
   }

   public void restoreFootContactPoints()
   {
      momentumBasedController.getContactState(contactableFoot).setContactFramePoints(originalContactPointPositions);
   }

   private void checkAnkleFlexionLimit()
   {
      OneDoFJoint oneDoFJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      double flexionLimit = oneDoFJoint.getJointLimitLower() + thresholdForAnkleLimitWatcher.getDoubleValue();

      isAnkleFlexionLimitReached.set(oneDoFJoint.getQ() < flexionLimit);
   }

   private final FramePoint2d tempFramePoint = new FramePoint2d();

   public void shrinkSupportFootContactPointsToToesIfNecessary()
   {
      if (!allowSupportFootShrink.getBooleanValue() || !isSupportFootShrinkEnabled.getBooleanValue())
         return;

      checkAnkleFlexionLimit();

      if (isAnkleFlexionLimitReached.getBooleanValue())
         footShrinkPercent.update(1.0);
      else
         footShrinkPercent.update(0.0);

      footShrinkPercent.set(MathTools.clipToMinMax(footShrinkPercent.getDoubleValue(), 0.0, maxFootShrinkInPercent.getDoubleValue()));

      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);

      double alpha = footShrinkPercent.getDoubleValue();

      for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint yoContactPoint = contactState.getContactPoints().get(i);
         double originalContactPointX = originalContactPointPositions.get(i).getX();
         yoContactPoint.getPosition2d(tempFramePoint);
         tempFramePoint.setX(alpha * contactPointMaxX.getDoubleValue() + (1.0 - alpha) * originalContactPointX);

         originalSupportFootPolygon.orthogonalProjection(tempFramePoint);
         yoContactPoint.setPosition2d(tempFramePoint);
      }
   }
}
