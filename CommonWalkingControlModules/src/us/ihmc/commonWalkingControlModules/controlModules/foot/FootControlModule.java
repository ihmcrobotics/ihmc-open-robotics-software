package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import org.ejml.alg.dense.mult.VectorVectorMult;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.kinematics.SpatialAccelerationProjector;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryGeneratorWithPushRecovery;
import us.ihmc.commonWalkingControlModules.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.NullspaceCalculator;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.CurrentLinearVelocityProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVelocityProvider;

public class FootControlModule
{
   private final YoVariableRegistry registry;
   private final ContactablePlaneBody contactableBody;

   public enum ConstraintType
   {
      FULL, HOLD_POSITION, HEEL_TOUCHDOWN, TOES_TOUCHDOWN, TOES, SWING, MOVE_STRAIGHT
   }

   private static final int NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT = 2;
   private static final boolean USE_SUPPORT_FOOT_HOLD_POSITION_STATE = true;
   private static final boolean USE_SUPPORT_DAMPING = true;
   private static final boolean USE_TOEOFF_FOOT_HOLD_POSITION = true;
   private static final boolean CORRECT_SWING_CONSIDERING_JOINT_LIMITS = true;
   private final double epsilonPointOnEdge = 1e-2;
   
   private boolean visualize = false;
   
   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   private final RigidBodySpatialAccelerationControlModule footSpatialAccelerationControlModule;
   private final SpatialAccelerationProjector spatialAccelerationProjector;
   private final GeometricJacobian jacobian;
   private final int jacobianId;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBody rootBody;
   private final EnumYoVariable<ConstraintType> requestedState;
   private final DoubleYoVariable desiredOnEdgeAngle;
   private final YoFrameLineSegment2d edgeToRotateAbout;
   private final BooleanYoVariable isCoPOnEdge;
   private final BooleanYoVariable doSingularityEscape;
   private final BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState;
   private final DoubleYoVariable minJacobianDeterminant;
   private final DoubleYoVariable nullspaceMultiplier;
   private final NullspaceCalculator nullspaceCalculator;
   private final DoubleYoVariable singularityEscapeNullspaceMultiplier;
   private final BooleanYoVariable isTrajectoryDone;
   private final DoubleYoVariable edgeControlOrientationDamping;
   private final int velocitySignForSingularityEscape;
   private final YoFrameQuaternion orientationFix;
   private final DoubleYoVariable jacobianDeterminant;
   private final BooleanYoVariable jacobianDeterminantInRange;
   private final BooleanYoVariable isUnconstrained;

   private final StateMachine<ConstraintType> stateMachine;
   // set every tick, so should not need to be YoVariablized
   private final SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F selectionMatrix;
   private final DenseMatrix64F jointVelocities;
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;

   private final BooleanYoVariable forceFootAccelerateIntoGround;
   private final DoubleYoVariable desiredZAccelerationIntoGround;

   private final int rootToFootJacobianId;

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameOrientation trajectoryOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final List<FramePoint2d> toeContactPoints, heelContactPoints;
   private final FramePoint2d singleToeContactPoint;

   private final OneDoFJoint hipYawJoint;
   private final OneDoFJoint kneeJoint;
   private final OneDoFJoint ankleRollJoint;
   private final OneDoFJoint anklePitchJoint;
   
   private final DoubleYoVariable footTouchDownGain;

   private final DoubleYoVariable swingKpXY, swingKpZ, swingKpOrientation, swingZetaXYZ, swingZetaOrientation;
   private final DoubleYoVariable holdKpx, holdKpy, holdKpz, holdKdz, holdKpRoll, holdKpPitch, holdKpYaw, holdZeta;
   private final DoubleYoVariable supportKdRoll, supportKdPitch, supportKdYaw;
   private final DoubleYoVariable toeOffKpx, toeOffKpy, toeOffKpz, toeOffKpRoll, toeOffKpPitch, toeOffKpYaw, toeOffZeta;
   
   private final BooleanYoVariable doFancyOnToesControl;
   
   private final BooleanYoVariable isHoldPositionRequested;
   
   private final double defaultCoefficientOfFriction;
   private final double lowerCoefficientOfFriction;
   private final DoubleYoVariable alphaCoefficientOfFriction;
   private final DoubleYoVariable desiredCoefficientOfFriction;
   private final AlphaFilteredYoVariable coefficientOfFrictionFiltered;
   
   //TODO yovariablelize all the inputs of the rigid body controller
   private final YoFramePoint yoDesiredPosition;
   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoFrameVector yoDesiredLinearAcceleration;
   
   private final FrameVector fullyConstrainedNormalContactVector;
   
   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;
   
   private final DoubleYoVariable controlledKneeToeOffQdd, kneeToeOffGain, kneeToeOffQDesired, kneeToeOffDamp;
   
   private final ReferenceFrame desiredOrientationFrame, correctedDesiredOrientationFrame;
   private final DynamicGraphicReferenceFrame desiredOrientationFrameGraphic, correctedDesiredOrientationFrameGraphic;
   
   private final DoubleYoVariable alphaShrinkFootSizeForToeOff;
   
   private final WalkingControllerParameters walkingControllerParameters;
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;

   private final BooleanYoVariable replanTrajectory;
   private final BooleanYoVariable trajectoryWasReplanned;
   private final YoVariableDoubleProvider swingTimeRemaining;

   public FootControlModule(double controlDT, ContactablePlaneBody contactablePlaneBody, int jacobianId, RobotSide robotSide,
         DoubleTrajectoryGenerator pitchTouchdownTrajectoryGenerator, DoubleProvider maximumTakeoffAngle,
         BooleanYoVariable requestHoldPosition, WalkingControllerParameters walkingControllerParameters,
         
         DoubleProvider swingTimeProvider, PositionProvider initialPositionProvider,
         PositionProvider finalPositionProvider, OrientationProvider initialOrientationProvider,
         OrientationProvider finalOrientationProvider, TrajectoryParametersProvider trajectoryParametersProvider,
         
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      this.walkingControllerParameters = walkingControllerParameters;
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
      
      this.jacobianId = jacobianId;
      jacobian = momentumBasedController.getJacobian(jacobianId);
      
      if (contactablePlaneBody.getRigidBody() != jacobian.getEndEffector())
         throw new RuntimeException("contactablePlaneBody does not match jacobian end effector!");
      
      RigidBody rigidBody = contactablePlaneBody.getRigidBody();
      String namePrefix = rigidBody.getName();
      DoubleYoVariable t = momentumBasedController.getYoTime();
      twistCalculator = momentumBasedController.getTwistCalculator();

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      
      alphaShrinkFootSizeForToeOff = new DoubleYoVariable(namePrefix + "AlphaShrinkFootSizeForToeOff", registry);
      alphaShrinkFootSizeForToeOff.set(0.0);
      
      this.contactableBody = contactablePlaneBody;
      this.hipYawJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_YAW);
      this.kneeJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE);
      this.ankleRollJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_ROLL);
      this.anklePitchJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      this.rootBody = twistCalculator.getRootBody();
      this.requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);
      this.momentumBasedController = momentumBasedController;
      this.isHoldPositionRequested = requestHoldPosition;
      
      fullyConstrainedNormalContactVector = new FrameVector(contactablePlaneBody.getPlaneFrame(), 0.0, 0.0, 1.0);

      taskspaceConstraintData.set(rootBody, contactablePlaneBody.getRigidBody());
      
      ReferenceFrame bodyFrame = contactablePlaneBody.getBodyFrame();
      rootToFootJacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootBody, jacobian.getEndEffector(), rootBody.getBodyFixedFrame());
      footSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, rigidBody, bodyFrame, controlDT, registry);
      spatialAccelerationProjector = new SpatialAccelerationProjector(namePrefix + "SpatialAccelerationProjector", registry);
      desiredOnEdgeAngle = new DoubleYoVariable(namePrefix + "DesiredOnEdgeAngle", registry);
      edgeToRotateAbout = new YoFrameLineSegment2d(namePrefix + "Edge", "", contactablePlaneBody.getPlaneFrame(), registry);
      isCoPOnEdge = new BooleanYoVariable(namePrefix + "IsCoPOnEdge", registry);
      doSingularityEscape = new BooleanYoVariable(namePrefix + "DoSingularityEscape", registry);
      waitSingularityEscapeBeforeTransitionToNextState = new BooleanYoVariable(namePrefix + "WaitSingularityEscapeBeforeTransitionToNextState", registry);
      minJacobianDeterminant = new DoubleYoVariable(namePrefix + "MinJacobianDeterminant", registry);
      jacobianDeterminant = new DoubleYoVariable(namePrefix + "JacobianDeterminant", registry);
      jacobianDeterminantInRange = new BooleanYoVariable(namePrefix + "JacobianDeterminantInRange", registry);
      nullspaceMultiplier = new DoubleYoVariable(namePrefix + "NullspaceMultiplier", registry);
      nullspaceCalculator = new NullspaceCalculator(jacobian.getNumberOfColumns(), true);
      singularityEscapeNullspaceMultiplier = new DoubleYoVariable(namePrefix + "SingularityEscapeNullspaceMultiplier", registry);
      isTrajectoryDone = new BooleanYoVariable(namePrefix + "IsTrajectoryDone", registry);
      edgeControlOrientationDamping = new DoubleYoVariable(namePrefix + "EdgeControlOrientationDamping", registry);
      isUnconstrained = new BooleanYoVariable(namePrefix + "IsUnconstrained", registry);
      
      forceFootAccelerateIntoGround = new BooleanYoVariable(namePrefix + "ForceFootAccelerateIntoGround", registry);
      desiredZAccelerationIntoGround = new DoubleYoVariable(namePrefix + "DesiredZAccelerationIntoGround", registry);
      desiredZAccelerationIntoGround.set(0.0); //-2.0);

      footTouchDownGain = new DoubleYoVariable(namePrefix + "footTouchDownGain", registry);
      footTouchDownGain.set(10);

      defaultCoefficientOfFriction = 0.8;
      lowerCoefficientOfFriction = 0.8;
      alphaCoefficientOfFriction = new DoubleYoVariable(namePrefix + "AlphaCoefficientOfFriction", registry);
      desiredCoefficientOfFriction = new DoubleYoVariable(namePrefix + "DesiredCoefficientOfFriction", registry);
      coefficientOfFrictionFiltered = new AlphaFilteredYoVariable(namePrefix + "CoefficientOfFriction", registry, alphaCoefficientOfFriction, desiredCoefficientOfFriction);

      alphaCoefficientOfFriction.set(0.8);
      desiredCoefficientOfFriction.set(defaultCoefficientOfFriction);
      coefficientOfFrictionFiltered.update();
      momentumBasedController.setPlaneContactCoefficientOfFriction(contactableBody, coefficientOfFrictionFiltered.getDoubleValue());
      
      swingKpXY = new DoubleYoVariable(namePrefix + "SwingKpXY", registry);
      swingKpZ = new DoubleYoVariable(namePrefix + "SwingKpZ", registry);
      swingKpOrientation = new DoubleYoVariable(namePrefix + "SwingKpOrientation", registry);
      swingZetaXYZ = new DoubleYoVariable(namePrefix + "SwingZetaXYZ", registry);
      swingZetaOrientation = new DoubleYoVariable(namePrefix + "SwingZetaOrientation", registry);

      holdKpx = new DoubleYoVariable(namePrefix + "HoldKpx", registry);
      holdKpy = new DoubleYoVariable(namePrefix + "HoldKpy", registry);
      holdKpz = new DoubleYoVariable(namePrefix + "HoldKpz", registry);
      holdKdz = new DoubleYoVariable(namePrefix + "HoldKdz", registry);
      holdKpRoll = new DoubleYoVariable(namePrefix + "HoldKpRoll", registry);
      holdKpPitch = new DoubleYoVariable(namePrefix + "HoldKpPitch", registry);
      holdKpYaw = new DoubleYoVariable(namePrefix + "HoldKpYaw", registry);
      holdZeta = new DoubleYoVariable(namePrefix + "HoldZeta", registry);

      supportKdRoll = new DoubleYoVariable(namePrefix + "SupportKdRoll", registry);
      supportKdRoll.set(20.0);
      supportKdPitch = new DoubleYoVariable(namePrefix + "SupportKdPitch", registry);
      supportKdYaw = new DoubleYoVariable(namePrefix + "SupportKdYaw", registry);
      
      kneeToeOffGain = new DoubleYoVariable(namePrefix + "KneeToeOffGain", registry);
      kneeToeOffQDesired   = new DoubleYoVariable(namePrefix + "KneeToeOffQDesired", registry);
      kneeToeOffDamp = new DoubleYoVariable(namePrefix + "KneeToeOffDamp", registry);
      controlledKneeToeOffQdd = new DoubleYoVariable(namePrefix + "KneeToeOffQdd_d", registry);
      // Work really fine in sim, but makes the robot do crazy toe-off
      kneeToeOffGain.set(0.0);
      kneeToeOffQDesired.set(0.7);
      kneeToeOffDamp.set(0.0);

      toeOffKpx = new DoubleYoVariable(namePrefix + "ToeOffKpx", registry);
      toeOffKpy = new DoubleYoVariable(namePrefix + "ToeOffKpy", registry);
      toeOffKpz = new DoubleYoVariable(namePrefix + "ToeOffKpz", registry);
      toeOffKpRoll = new DoubleYoVariable(namePrefix + "ToeOffKpRoll", registry);
      toeOffKpPitch = new DoubleYoVariable(namePrefix + "ToeOffKpPitch", registry);
      toeOffKpYaw = new DoubleYoVariable(namePrefix + "ToeOffKpYaw", registry);
      toeOffZeta = new DoubleYoVariable(namePrefix + "ToeOffZeta", registry);

      doFancyOnToesControl = new BooleanYoVariable(contactablePlaneBody.getName() + "DoFancyOnToesControl", registry);
      if (walkingControllerParameters.isRunningOnRealRobot())
         doFancyOnToesControl.set(false);
      else
         doFancyOnToesControl.set(true);
      
      stateMachine = new StateMachine<ConstraintType>(namePrefix + "State", namePrefix + "SwitchTime", ConstraintType.class, t, registry);

      orientationFix = new YoFrameQuaternion(namePrefix + "OrientationFix", ReferenceFrame.getWorldFrame(), registry);

      // TODO: make this be able to do other selection matrices
      selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      edgeControlOrientationDamping.set(100.0);

      // TODO: pass in:
      this.velocitySignForSingularityEscape = 1;
      jointVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jacobian.getJointsInOrder()), 1);

      toeContactPoints = getEdgeContactPoints2d(contactablePlaneBody, ConstraintType.TOES);
      heelContactPoints = getEdgeContactPoints2d(contactablePlaneBody, ConstraintType.HEEL_TOUCHDOWN);
      
      singleToeContactPoint = new FramePoint2d(toeContactPoints.get(0).getReferenceFrame());
      singleToeContactPoint.interpolate(toeContactPoints.get(0), toeContactPoints.get(1), 0.5);

      yoDesiredLinearVelocity = new YoFrameVector(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      yoDesiredLinearAcceleration = new YoFrameVector(namePrefix + "DesiredLinearAcceleration", worldFrame, registry);
      yoDesiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", worldFrame, registry);
      yoDesiredPosition.setToNaN();
      
      setupContactStatesMap();
      parentRegistry.addChild(registry);
      
      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactablePlaneBody, robotSide, walkingControllerParameters, momentumBasedController, dynamicGraphicObjectsListRegistry, registry);
      
      replanTrajectory = new BooleanYoVariable(namePrefix + "replanTrajectory", registry);
      trajectoryWasReplanned = new BooleanYoVariable(namePrefix + "trajectoryWasReplanned", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);
      
      visualize = visualize && dynamicGraphicObjectsListRegistry != null;
      
      if (visualize)
      {
         desiredOrientationFrame = new ReferenceFrame(namePrefix + "DesiredOrientationFrame", worldFrame)
         {
            private final FramePoint footPosition = new FramePoint();
            private final Vector3d footTranslationToWorld = new Vector3d();
            private static final long serialVersionUID = -4968527479367321388L;

            @Override
            public void updateTransformToParent(Transform3D transformToParent)
            {
               trajectoryOrientation.getTransform3D(transformToParent);
               footPosition.setToZero(getFootFrame());
               footPosition.changeFrame(worldFrame);
               footPosition.get(footTranslationToWorld);
               transformToParent.setTranslation(footTranslationToWorld);
            }
         };

         correctedDesiredOrientationFrame = new ReferenceFrame(namePrefix + "CorrectedDesiredOrientationFrame", worldFrame)
         {
            private final FramePoint footPosition = new FramePoint();
            private final Vector3d footTranslationToWorld = new Vector3d();
            private static final long serialVersionUID = 2338083143740929570L;

            @Override
            public void updateTransformToParent(Transform3D transformToParent)
            {
               desiredOrientation.getTransform3D(transformToParent);
               footPosition.setToZero(getFootFrame());
               footPosition.changeFrame(worldFrame);
               footPosition.get(footTranslationToWorld);
               transformToParent.setTranslation(footTranslationToWorld);
            }
         };
         
         desiredOrientationFrameGraphic = new DynamicGraphicReferenceFrame(desiredOrientationFrame, registry, 0.2, YoAppearance.Red());
         correctedDesiredOrientationFrameGraphic = new DynamicGraphicReferenceFrame(correctedDesiredOrientationFrame, registry, 0.2, YoAppearance.Green());
         desiredOrientationFrameGraphic.hideGraphicObject();
         correctedDesiredOrientationFrameGraphic.hideGraphicObject();

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("JointLimitCorrection", desiredOrientationFrameGraphic);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("JointLimitCorrection", correctedDesiredOrientationFrameGraphic);
      }
      else
      {
         desiredOrientationFrame = null;
         correctedDesiredOrientationFrame = null;
         
         desiredOrientationFrameGraphic = null;
         correctedDesiredOrientationFrameGraphic = null;
      }
      
      YoVelocityProvider finalVelocityProvider =
            new YoVelocityProvider(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), registry);
      finalVelocityProvider.set(new Vector3d(0.0, 0.0, walkingControllerParameters.getDesiredTouchdownVelocity()));
      
      VectorProvider initialVelocityProvider = new CurrentLinearVelocityProvider(
            momentumBasedController.getReferenceFrames().getFootFrame(robotSide),
            momentumBasedController.getFullRobotModel().getFoot(robotSide), twistCalculator);
      
      List<FootControlState> states = new ArrayList<FootControlState>();
      states.add(new TouchdownOnToesState(pitchTouchdownTrajectoryGenerator));
      states.add(new TouchdownOnHeelState(pitchTouchdownTrajectoryGenerator));
      states.add(new OnToesState(maximumTakeoffAngle));
      states.add(new FullyConstrainedState());
      states.add(new HoldPositionState());
      states.add(new SwingState(swingTimeProvider, initialPositionProvider, initialVelocityProvider,
            finalPositionProvider, finalVelocityProvider, initialOrientationProvider,
            finalOrientationProvider, trajectoryParametersProvider));
      states.add(new MoveStraightState(swingTimeProvider, initialPositionProvider,
            finalPositionProvider, initialOrientationProvider, finalOrientationProvider));

      setUpStateMachine(states);
   }

   public void setMaxAccelerationAndJerk(double maxPositionAcceleration, double maxPositionJerk, 
         double maxOrientationAcceleration, double maxOrientationJerk)
   {
      footSpatialAccelerationControlModule.setPositionMaxAccelerationAndJerk(maxPositionAcceleration, maxPositionJerk);
      footSpatialAccelerationControlModule.setOrientationMaxAccelerationAndJerk(maxOrientationAcceleration, maxOrientationJerk);
      
   }
   
   public void setSwingGains(double swingKpXY, double swingKpZ, double swingKpOrientation, double swingZetaXYZ, double swingZetaOrientation)
   {
      this.swingKpXY.set(swingKpXY);
      this.swingKpZ.set(swingKpZ);
      this.swingKpOrientation.set(swingKpOrientation);
      this.swingZetaXYZ.set(swingZetaXYZ);
      this.swingZetaOrientation.set(swingZetaOrientation);
   }
   
   public void setHoldGains(double holdKpXY, double holdKpOrientation, double holdZeta)
   {
      this.holdZeta.set(holdZeta);
      this.holdKpx.set(holdKpXY);
      this.holdKpy.set(holdKpXY);
      
      this.holdKpz.set(0.0);
      this.holdKdz.set(GainCalculator.computeDerivativeGain(this.holdKpz.getDoubleValue(), this.holdZeta.getDoubleValue()));
      
      this.holdKpRoll.set(holdKpOrientation);
      this.holdKpPitch.set(holdKpOrientation);
      this.holdKpYaw.set(holdKpOrientation);
   }

   public void setToeOffGains(double toeOffKpXY, double toeOffKpOrientation, double toeOffZeta)
   {
      this.toeOffZeta.set(toeOffZeta);
      this.toeOffKpx.set(toeOffKpXY);
      this.toeOffKpy.set(toeOffKpXY);
      
      this.toeOffKpz.set(0.0);
      
      this.toeOffKpRoll.set(toeOffKpOrientation);
      this.toeOffKpPitch.set(toeOffKpOrientation);
      this.toeOffKpYaw.set(toeOffKpOrientation);
   }
   
   private void setupContactStatesMap()
   {
      boolean[] falses = new boolean[contactableBody.getTotalNumberOfContactPoints()];
      Arrays.fill(falses, false);
      boolean[] trues = new boolean[contactableBody.getTotalNumberOfContactPoints()];
      Arrays.fill(trues, true);

      contactStatesMap.put(ConstraintType.SWING, falses);
      contactStatesMap.put(ConstraintType.MOVE_STRAIGHT, falses);
      contactStatesMap.put(ConstraintType.FULL, trues);
      contactStatesMap.put(ConstraintType.HOLD_POSITION, trues);
      contactStatesMap.put(ConstraintType.HEEL_TOUCHDOWN, getOnEdgeContactPointStates(contactableBody, ConstraintType.HEEL_TOUCHDOWN));
      contactStatesMap.put(ConstraintType.TOES, trues);
      contactStatesMap.put(ConstraintType.TOES_TOUCHDOWN, contactStatesMap.get(ConstraintType.TOES));
   }

   private void setUpStateMachine(List<FootControlState> states)
   {
      for (FootControlState state : states)
      {
         for (FootControlState stateToTransitionTo : states)
         {
            FootStateTransitionCondition footStateTransitionCondition = new FootStateTransitionCondition(stateToTransitionTo);
            state.addStateTransition(new StateTransition<ConstraintType>(stateToTransitionTo.getStateEnum(), footStateTransitionCondition,
                  new FootStateTransitionAction()));
         }
      }

      for (State<ConstraintType> state : states)
      {
         stateMachine.addState(state);
      }

      stateMachine.setCurrentState(ConstraintType.FULL);
   }
   
   public void replanTrajectory(double swingTimeRemaining)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
      this.replanTrajectory.set(true);
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

   public void doSingularityEscapeBeforeTransitionToNextState()
   {
      doSingularityEscape(true);
      waitSingularityEscapeBeforeTransitionToNextState.set(true);
   }

   public double getJacobianDeterminant()
   {
      return jacobianDeterminant.getDoubleValue();
   }

   public boolean isInSingularityNeighborhood()
   {
      return jacobianDeterminantInRange.getBooleanValue();
   }

   public void forceConstrainedFootToAccelerateIntoGround(boolean accelerateIntoGround)
   {
      forceFootAccelerateIntoGround.set(accelerateIntoGround);
   }

   public void setParameters(double minJacobianDeterminantForSingularityEscape, double singularityEscapeNullspaceMultiplier)
   {
      this.minJacobianDeterminant.set(minJacobianDeterminantForSingularityEscape);
      this.singularityEscapeNullspaceMultiplier.set(singularityEscapeNullspaceMultiplier);
   }

   public void setContactState(ConstraintType constraintType)
   {
      setContactState(constraintType, null);
   }

   public void setContactState(ConstraintType constraintType, FrameVector normalContactVector)
   {
      if (constraintType == ConstraintType.HOLD_POSITION || constraintType == ConstraintType.FULL)
      {
         if (constraintType == ConstraintType.HOLD_POSITION)
            System.out.println("Warning: HOLD_POSITION state is handled internally.");
         
         if (isHoldPositionRequested != null && isHoldPositionRequested.getBooleanValue())
            constraintType = ConstraintType.HOLD_POSITION;
         else
            constraintType = ConstraintType.FULL;
         
         if (normalContactVector != null)
            fullyConstrainedNormalContactVector.setIncludingFrame(normalContactVector);
         else
            fullyConstrainedNormalContactVector.setIncludingFrame(contactableBody.getPlaneFrame(), 0.0, 0.0, 1.0);
      }
      
      momentumBasedController.setPlaneContactState(contactableBody, contactStatesMap.get(constraintType), normalContactVector);

      if (getCurrentConstraintType() == constraintType) // Use resetCurrentState() for such case
         return;
      
      requestedState.set(constraintType);
   }

   public ConstraintType getCurrentConstraintType()
   {
      return stateMachine.getCurrentStateEnum();
   }

   private abstract class OnEdgeState extends FootControlState
   {
      private final DoubleTrajectoryGenerator onEdgePitchAngleTrajectoryGenerator;
      private final List<FramePoint2d> edgeContactPoints;
      private final List<FramePoint> desiredEdgeContactPositions;
      private final DoubleProvider maximumPitchAngleOnEdge;
      protected final boolean useTrajectory, useMaximumPitchAngle;
      // Just to make sure we're not trying to do singularity escape
      // (the MotionConstraintHandler crashes when using point jacobian and singularity escape)
      private final DenseMatrix64F onEdgeNullspaceMultipliers = new DenseMatrix64F(0, 1);
      
      private final Twist footTwist = new Twist();
      
      private double desiredYawToHold = 0.0;
      private double desiredRollToHold = 0.0;
      private final double[] tempYawPitchRoll = new double[3];

      private final FramePoint contactPointPosition = new FramePoint();
      private final FrameVector contactPointLinearVelocity = new FrameVector();
      private final FramePoint proportionalPart = new FramePoint();
      private final FrameVector derivativePart = new FrameVector();
      
      private double toeOffKdx = GainCalculator.computeDerivativeGain(toeOffKpx.getDoubleValue(), toeOffZeta.getDoubleValue());      
      private double toeOffKdy = GainCalculator.computeDerivativeGain(toeOffKpy.getDoubleValue(), toeOffZeta.getDoubleValue());      
      private double toeOffKdz = GainCalculator.computeDerivativeGain(toeOffKpz.getDoubleValue(), toeOffZeta.getDoubleValue());

      public OnEdgeState(ConstraintType stateEnum, List<FramePoint2d> edgeContactPoints, DoubleProvider maximumPitchAngleOnEdge)
      {
         super(stateEnum, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
               footSpatialAccelerationControlModule);
         this.onEdgePitchAngleTrajectoryGenerator = null;
         this.maximumPitchAngleOnEdge = maximumPitchAngleOnEdge;
         useTrajectory = false;
         useMaximumPitchAngle = maximumPitchAngleOnEdge != null;

         if (edgeContactPoints.size() != NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT)
            throw new RuntimeException("Number of contacts not handled for OnEdgeState: " + edgeContactPoints.size());

         this.edgeContactPoints = edgeContactPoints;
         desiredEdgeContactPositions = new ArrayList<FramePoint>();
         for (int i = 0; i < 2; i++)
            desiredEdgeContactPositions.add(edgeContactPoints.get(i).toFramePoint());
      }

      public void doTransitionIntoAction()
      {
         toeOffKdx = GainCalculator.computeDerivativeGain(toeOffKpx.getDoubleValue(), toeOffZeta.getDoubleValue());      
         toeOffKdy = GainCalculator.computeDerivativeGain(toeOffKpy.getDoubleValue(), toeOffZeta.getDoubleValue());      
         toeOffKdz = GainCalculator.computeDerivativeGain(toeOffKpz.getDoubleValue(), toeOffZeta.getDoubleValue());
         
         edgeToRotateAbout.set(edgeContactPoints.get(0), edgeContactPoints.get(1));
         for (int i = 0; i < 2; i++)
         {
            desiredEdgeContactPositions.get(i).setIncludingFrame(edgeContactPoints.get(i).getReferenceFrame(), edgeContactPoints.get(i).getX(), edgeContactPoints.get(i).getY(), 0.0);
            desiredEdgeContactPositions.get(i).changeFrame(rootBody.getBodyFixedFrame());
         }
         
         if (onEdgePitchAngleTrajectoryGenerator != null)
         {
            onEdgePitchAngleTrajectoryGenerator.initialize();
         }
         
         desiredOrientation.setToZero(contactableBody.getBodyFrame());
         desiredOrientation.changeFrame(worldFrame);
         desiredYawToHold = desiredOrientation.getYaw();
         desiredRollToHold = desiredOrientation.getRoll();
      }

      public void doSpecificAction()
      {
         desiredOrientation.setToZero(contactableBody.getBodyFrame());
         desiredOrientation.changeFrame(worldFrame);
         desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

         twistCalculator.packRelativeTwist(footTwist, rootBody, contactableBody.getRigidBody());
         footTwist.changeFrame(contactableBody.getBodyFrame());

         boolean blockToMaximumPitch = useMaximumPitchAngle && tempYawPitchRoll[1] > maximumPitchAngleOnEdge.getValue();

         double footPitch = 0.0, footPitchd = 0.0, footPitchdd = 0.0;

         desiredPosition.setToZero(contactableBody.getBodyFrame());
         desiredPosition.changeFrame(worldFrame);

         if (useTrajectory)
         {
            onEdgePitchAngleTrajectoryGenerator.compute(getTimeInCurrentState());
            footPitch = onEdgePitchAngleTrajectoryGenerator.getValue();
            footPitchd = onEdgePitchAngleTrajectoryGenerator.getVelocity();
            footPitchdd = onEdgePitchAngleTrajectoryGenerator.getAcceleration();
            desiredOnEdgeAngle.set(footPitch);
         }
         else if (useMaximumPitchAngle)
         {
            if (blockToMaximumPitch)
            {
               footPitch = maximumPitchAngleOnEdge.getValue();
               footPitchd = 0.0;
               footPitchdd = 0.0;
            }
            else
            {
               footPitch = desiredOrientation.getPitch();
               footPitchd = footTwist.getAngularPartY();
               footPitchdd = 0.0;
            }
         }

         if (useTrajectory || useMaximumPitchAngle)
         {
            if (USE_TOEOFF_FOOT_HOLD_POSITION)
               desiredOrientation.setYawPitchRoll(desiredYawToHold, footPitch, desiredRollToHold);
            else
               desiredOrientation.setYawPitchRoll(tempYawPitchRoll[0], footPitch, tempYawPitchRoll[2]);

            desiredLinearVelocity.setToZero(worldFrame);
            desiredAngularVelocity.setIncludingFrame(contactableBody.getBodyFrame(), 0.0, footPitchd, 0.0);
            desiredAngularVelocity.changeFrame(worldFrame);

            desiredLinearAcceleration.setToZero(worldFrame);
            desiredAngularAcceleration.setIncludingFrame(contactableBody.getBodyFrame(), 0.0, footPitchdd, 0.0);
            desiredAngularAcceleration.changeFrame(worldFrame);

            accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity,
                  desiredAngularVelocity, desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
            accelerationControlModule.packAcceleration(footAcceleration);
         }

         if (momentumBasedController.isUsingOptimizationMomentumControlModule())
         {
            
            for (int i = 0; i < edgeContactPoints.size(); i++)
            {
               FramePoint2d contactPoint2d = edgeContactPoints.get(i);
               contactPointPosition.setIncludingFrame(contactPoint2d.getReferenceFrame(), contactPoint2d.getX(), contactPoint2d.getY(), 0.0);
               
               contactPointPosition.changeFrame(footTwist.getBaseFrame());
               footTwist.changeFrame(footTwist.getBaseFrame());
               footTwist.packVelocityOfPointFixedInBodyFrame(contactPointLinearVelocity, contactPointPosition);
               contactPointPosition.changeFrame(rootBody.getBodyFixedFrame());

               proportionalPart.changeFrame(rootBody.getBodyFixedFrame());
               proportionalPart.sub(desiredEdgeContactPositions.get(i), contactPointPosition);
               proportionalPart.scale(toeOffKpx.getDoubleValue(), toeOffKpy.getDoubleValue(), toeOffKpz.getDoubleValue());
               
               derivativePart.setToZero(rootBody.getBodyFixedFrame());
               derivativePart.sub(contactPointLinearVelocity);
               derivativePart.scale(toeOffKdx, toeOffKdy, toeOffKdz);
               
               desiredLinearAcceleration.setToZero(rootBody.getBodyFixedFrame());
               desiredLinearAcceleration.add(proportionalPart);
               desiredLinearAcceleration.add(derivativePart);

               if (forceFootAccelerateIntoGround.getBooleanValue())
                  desiredLinearAcceleration.setZ(desiredZAccelerationIntoGround.getDoubleValue());
               
               momentumBasedController.setDesiredPointAcceleration(rootToFootJacobianId, contactPointPosition, desiredLinearAcceleration);
            }

            if (useTrajectory || useMaximumPitchAngle)
            {
               FrameVector2d axisOfRotation2d = edgeToRotateAbout.getFrameLineSegment2d().getVectorCopy();
               FrameVector axisOfRotation = new FrameVector(axisOfRotation2d.getReferenceFrame(), axisOfRotation2d.getX(), axisOfRotation2d.getY(), 0.0);
               axisOfRotation.normalize();
               axisOfRotation.changeFrame(footAcceleration.getExpressedInFrame());

               selectionMatrix.reshape(2, SpatialMotionVector.SIZE);
               selectionMatrix.set(0, 0, axisOfRotation.getX());
               selectionMatrix.set(0, 1, axisOfRotation.getY());
               selectionMatrix.set(0, 2, axisOfRotation.getZ());
               selectionMatrix.set(1, 0, 0.0);
               selectionMatrix.set(1, 1, 0.0);
               selectionMatrix.set(1, 2, 1.0);

               footAcceleration.changeBodyFrameNoRelativeAcceleration(jacobian.getEndEffectorFrame());
               footAcceleration.changeFrameNoRelativeMotion(jacobian.getEndEffectorFrame());
               taskspaceConstraintData.set(footAcceleration, onEdgeNullspaceMultipliers, selectionMatrix);
               momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
            }
         }
         else
         {
            spatialAccelerationProjector.projectAcceleration(footAcceleration, edgeToRotateAbout.getFrameLineSegment2d());

            setTaskspaceConstraint(footAcceleration);
         }
      }

      public void doTransitionOutOfAction()
      {
         // TODO: kind of a hack
         selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
      }
   }

   private class TouchdownOnHeelState extends OnEdgeState
   {
      public TouchdownOnHeelState(DoubleTrajectoryGenerator pitchTouchdownTrajectoryGenerator)
      {
         super(ConstraintType.HEEL_TOUCHDOWN, heelContactPoints, pitchTouchdownTrajectoryGenerator);
      }

      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();
         setTouchdownOnEdgeGains();
      }
   }

   private class TouchdownOnToesState extends OnEdgeState
   {
      public TouchdownOnToesState(DoubleTrajectoryGenerator pitchTouchdownTrajectoryGenerator)
      {
         super(ConstraintType.TOES_TOUCHDOWN, heelContactPoints, pitchTouchdownTrajectoryGenerator);
      }

      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();
         setTouchdownOnEdgeGains();
      }
   }

   private class OnToesState extends OnEdgeState
   {
      private final YoPlaneContactState contactState = momentumBasedController.getContactState(contactableBody);
      private final List<YoContactPoint> contactPoints = contactState.getContactPoints();
      private final List<FramePoint> originalContactPointPositions;

      public OnToesState(DoubleProvider maximumTakeoffAngle)
      {
         super(ConstraintType.TOES, toeContactPoints, maximumTakeoffAngle);

         originalContactPointPositions = new ArrayList<FramePoint>(contactPoints.size());
         copyOriginalContactPointPositions();
      }
      
      public void doSpecificAction()
      {
         super.doSpecificAction();
         
         shrinkFootSizeToMidToe();

         // TODO Hack to do singularity escape since the motion constraint handler doesn't handle it with the given selection matrix 
         controlledKneeToeOffQdd.set(kneeToeOffGain.getDoubleValue() * Math.max(0.0, kneeToeOffQDesired.getDoubleValue() - kneeJoint.getQ()) - kneeToeOffDamp.getDoubleValue() * kneeJoint.getQd());
         momentumBasedController.setOneDoFJointAcceleration(kneeJoint, controlledKneeToeOffQdd.getDoubleValue());
      }

      private void copyOriginalContactPointPositions()
      {
         for (int i = 0; i < contactPoints.size(); i++)
         {
            originalContactPointPositions.add(new FramePoint(contactPoints.get(i).getPosition()));
         }
      }

      private void resetContactPointPositions()
      {
         for (int i = 0; i < contactPoints.size(); i++)
         {
            contactPoints.get(i).getPosition().set(originalContactPointPositions.get(i));
         }
      }
      
      private void shrinkFootSizeToMidToe()
      {
         double alphaShrink = alphaShrinkFootSizeForToeOff.getDoubleValue();
         for (int i = 0; i < contactPoints.size(); i++)
         {
            FramePoint position = contactPoints.get(i).getPosition();
            position.setX(alphaShrink * position.getX() + (1.0 - alphaShrink) * singleToeContactPoint.getX());
            position.setY(alphaShrink * position.getY() + (1.0 - alphaShrink) * singleToeContactPoint.getY());
         }
      }
      
      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();

         if (useMaximumPitchAngle)
         {
            setOnToesFreeMotionGains();
         }
         else
         {
            setStanceControlGains();
         }
         
         //alphaShrinkFootSizeForToeOff
         
         momentumBasedController.setPlaneContactState(contactableBody, contactStatesMap.get(ConstraintType.TOES));
      }

      public void doTransitionOutOfAction()
      {
         super.doTransitionOutOfAction();

         resetContactPointPositions();
         
         controlledKneeToeOffQdd.set(0.0);
      }
   }

   private class FullyConstrainedState extends FootControlState
   {
      public FullyConstrainedState()
      {
         super(ConstraintType.FULL, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
               footSpatialAccelerationControlModule);
      }

      public void doTransitionIntoAction()
      {
         momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, fullyConstrainedNormalContactVector);
         desiredCoefficientOfFriction.set(defaultCoefficientOfFriction);
      }
      
      private void setFullyConstrainedStateGains()
      {
         setGains(0.0, 0.0, 0.0);
         accelerationControlModule.setOrientationDerivativeGains(supportKdRoll.getDoubleValue(), supportKdPitch.getDoubleValue(), supportKdYaw.getDoubleValue());
      }

      public void doSpecificAction()
      {
         if (doFancyOnToesControl.getBooleanValue())
            determineCoPOnEdge();

         if (USE_SUPPORT_FOOT_HOLD_POSITION_STATE)
         {
            if (isCoPOnEdge.getBooleanValue() && doFancyOnToesControl.getBooleanValue())
               requestedState.set(ConstraintType.HOLD_POSITION);
            else if (isHoldPositionRequested != null && isHoldPositionRequested.getBooleanValue())
               requestedState.set(ConstraintType.HOLD_POSITION);
         }
         
         desiredCoefficientOfFriction.set(defaultCoefficientOfFriction);

         if (!USE_SUPPORT_DAMPING)
         {
            footAcceleration.setToZero(contactableBody.getBodyFrame(), rootBody.getBodyFixedFrame(), contactableBody.getBodyFrame());
         }
         else
         {
            setFullyConstrainedStateGains();
            
            desiredPosition.setToZero(contactableBody.getBodyFrame());
            desiredPosition.changeFrame(worldFrame);

            desiredOrientation.setToZero(contactableBody.getBodyFrame());
            desiredOrientation.changeFrame(worldFrame);

            desiredLinearVelocity.setToZero(worldFrame);
            desiredAngularVelocity.setToZero(worldFrame);

            desiredLinearAcceleration.setToZero(worldFrame);
            desiredAngularAcceleration.setToZero(worldFrame);

            accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
                  desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
            accelerationControlModule.packAcceleration(footAcceleration);
         }

         if (forceFootAccelerateIntoGround.getBooleanValue())
            footAcceleration.setLinearPartZ(footAcceleration.getLinearPartZ() + desiredZAccelerationIntoGround.getDoubleValue());
         
         setTaskspaceConstraint(footAcceleration);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         desiredCoefficientOfFriction.set(defaultCoefficientOfFriction);
      }
   }

   private class HoldPositionState extends FootControlState
   {
      private final FrameVector holdPositionNormalContactVector = new FrameVector();
      
      public HoldPositionState()
      {
         super(ConstraintType.HOLD_POSITION, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
               footSpatialAccelerationControlModule);
      }

      public void doTransitionIntoAction()
      {
         // Remember the previous contact normal, in case the foot leaves the ground and rotates
         holdPositionNormalContactVector.setIncludingFrame(fullyConstrainedNormalContactVector);
         holdPositionNormalContactVector.changeFrame(worldFrame);
         momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, holdPositionNormalContactVector);

         desiredCoefficientOfFriction.set(lowerCoefficientOfFriction);

         desiredPosition.setToZero(contactableBody.getBodyFrame());
         desiredPosition.changeFrame(worldFrame);

         desiredOrientation.setToZero(contactableBody.getBodyFrame());
         desiredOrientation.changeFrame(worldFrame);
         
         desiredLinearVelocity.setToZero(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         desiredLinearAcceleration.setToZero(worldFrame);
         desiredAngularAcceleration.setToZero(worldFrame);
      }

      public void doSpecificAction()
      {
         setHoldPositionStateGains();
         
         determineCoPOnEdge();

         if (!isCoPOnEdge.getBooleanValue() && (isHoldPositionRequested == null || !isHoldPositionRequested.getBooleanValue()))
            requestedState.set(ConstraintType.FULL);
         
         yoDesiredPosition.set(desiredPosition);
         accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
         accelerationControlModule.packAcceleration(footAcceleration);

         if (forceFootAccelerateIntoGround.getBooleanValue())
            footAcceleration.setLinearPartZ(footAcceleration.getLinearPartZ() + desiredZAccelerationIntoGround.getDoubleValue());
         
         setTaskspaceConstraint(footAcceleration);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         desiredCoefficientOfFriction.set(defaultCoefficientOfFriction);

         yoDesiredPosition.setToNaN();
      }
   }

   private class SwingState extends UnconstrainedState
   {
      private final boolean visualizeSwingTrajectory = true;

      private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;

      private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

      public SwingState(DoubleProvider swingTimeProvider, PositionProvider initialPositionProvider, VectorProvider initialVelocityProvider,
            PositionProvider finalPositionProvider, VectorProvider finalVelocityProvider, OrientationProvider initialOrientationProvider,
            OrientationProvider finalOrientationProvider, TrajectoryParametersProvider trajectoryParametersProvider)
      {
         super(ConstraintType.SWING);

         RigidBody rigidBody = contactableBody.getRigidBody();
         String namePrefix = rigidBody.getName();

         ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
         ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

         if (trajectoryParametersProvider == null)
         {
            trajectoryParametersProvider = new TrajectoryParametersProvider(new SimpleTwoWaypointTrajectoryParameters());
         }

         PositionTrajectoryGenerator swingTrajectoryGenerator = new TwoWaypointPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame,
               swingTimeProvider, initialPositionProvider, initialVelocityProvider, finalPositionProvider, finalVelocityProvider,
               trajectoryParametersProvider, registry, dynamicGraphicObjectsListRegistry, walkingControllerParameters, visualizeSwingTrajectory);

         PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
               finalPositionProvider, finalVelocityProvider, swingTimeProvider, registry);

         PositionTrajectoryGenerator pushRecoverySwingTrajectoryGenerator = new TwoWaypointTrajectoryGeneratorWithPushRecovery(
               namePrefix + "PushRecoverySwing", worldFrame, swingTimeProvider, swingTimeRemaining, initialPositionProvider,
               initialVelocityProvider, finalPositionProvider, finalVelocityProvider, trajectoryParametersProvider, registry,
               dynamicGraphicObjectsListRegistry, swingTrajectoryGenerator, walkingControllerParameters, visualizeSwingTrajectory);

         positionTrajectoryGenerators.add(swingTrajectoryGenerator);
         positionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

         pushRecoveryPositionTrajectoryGenerators.add(pushRecoverySwingTrajectoryGenerator);
         pushRecoveryPositionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

         positionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(positionTrajectoryGenerators, namePrefix, registry);

         pushRecoveryPositionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(pushRecoveryPositionTrajectoryGenerators, namePrefix
               + "PushRecoveryTrajectoryGenerator", registry);

         orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "SwingFootOrientation", worldFrame,
               swingTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);
      }

      protected void initializeTrajectory()
      {
         positionTrajectoryGenerator.initialize();
         orientationTrajectoryGenerator.initialize();

         trajectoryWasReplanned.set(false);
         replanTrajectory.set(false);
      }

      protected void computeAndPackTrajectory()
      {
         if (replanTrajectory.getBooleanValue()) // This seems like a bad place for this?
         {
            pushRecoveryPositionTrajectoryGenerator.initialize();
            replanTrajectory.set(false);
            trajectoryWasReplanned.set(true);
         }

         if (!trajectoryWasReplanned.getBooleanValue())
         {
            boolean isDone = positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
            isTrajectoryDone.set(isDone);

            positionTrajectoryGenerator.compute(getTimeInCurrentState());

            positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         }
         else
         {
            boolean isDone = pushRecoveryPositionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
            isTrajectoryDone.set(isDone);

            pushRecoveryPositionTrajectoryGenerator.compute(getTimeInCurrentState());

            pushRecoveryPositionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         }

         orientationTrajectoryGenerator.compute(getTimeInCurrentState());
         orientationTrajectoryGenerator.packAngularData(trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      }
   }

   private class MoveStraightState extends UnconstrainedState
   {
      private StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator;
      private OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

      public MoveStraightState(DoubleProvider footTrajectoryTimeProvider, PositionProvider initialPositionProvider,
            PositionProvider finalPositionProvider, OrientationProvider initialOrientationProvider, OrientationProvider finalOrientationProvider)
      {
         super(ConstraintType.MOVE_STRAIGHT);

         RigidBody rigidBody = contactableBody.getRigidBody();
         String namePrefix = rigidBody.getName();

         positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix + "FootPosition", worldFrame, footTrajectoryTimeProvider,
               initialPositionProvider, finalPositionProvider, registry);

         orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "Orientation", worldFrame, footTrajectoryTimeProvider,
               initialOrientationProvider, finalOrientationProvider, registry);
      }

      protected void initializeTrajectory()
      {
         positionTrajectoryGenerator.initialize();
         orientationTrajectoryGenerator.initialize();
      }

      protected void computeAndPackTrajectory()
      {
         boolean isDone = positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
         isTrajectoryDone.set(isDone);

         positionTrajectoryGenerator.compute(getTimeInCurrentState());
         orientationTrajectoryGenerator.compute(getTimeInCurrentState());

         positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.packAngularData(trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      }
   }

   /**
    * The unconstrained state is used if the foot is moved free in space without constrains. Depending on the type of trajectory
    * this can either be a movement along a straight line or (in case of walking) a swing motion.
    * 
    * E.g. the MoveStraightState and the SwingState extend this class and implement the trajectory related methods.
    */
   private abstract class UnconstrainedState extends FootControlState
   {
      public UnconstrainedState(ConstraintType constraintType)
      {
         super(constraintType, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
               footSpatialAccelerationControlModule);
      }

      /**
       * initializes all the trajectories
       */
      protected abstract void initializeTrajectory();

      /**
       * compute the and pack the following variables:
       * desiredPosition, desiredLinearVelocity, desiredLinearAcceleration,
       * trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration
       */
      protected abstract void computeAndPackTrajectory();

      public void doTransitionIntoAction()
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(true);

         accelerationControlModule.reset();

         isCoPOnEdge.set(false);
         initializeTrajectory();

         isTrajectoryDone.set(false);
         isUnconstrained.set(true);

         setSwingControlGains(swingKpXY.getDoubleValue(), swingKpZ.getDoubleValue(), swingKpOrientation.getDoubleValue(), swingZetaXYZ.getDoubleValue(), swingZetaOrientation.getDoubleValue());

         if (visualize)
         {
            desiredOrientationFrameGraphic.showGraphicObject();
            correctedDesiredOrientationFrameGraphic.showGraphicObject();
         }
      }

      public void doSpecificAction()
      {
         if (doSingularityEscape.getBooleanValue())
         {
            initializeTrajectory();
         }

         computeAndPackTrajectory();

         yoDesiredPosition.set(desiredPosition);

         desiredOrientation.setAndChangeFrame(trajectoryOrientation);

         if (CORRECT_SWING_CONSIDERING_JOINT_LIMITS)
            correctInputsAccordingToJointLimits();

         legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectoryForSingularityAvoidance(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);

         accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
         accelerationControlModule.packAcceleration(footAcceleration);

         setTaskspaceConstraint(footAcceleration);

         updateVisualizers();
      }

      private final double[] desiredYawPitchRoll = new double[3];
      private final double epsilon = 1e-3;

      // TODO Pretty much hackish...
      private void correctInputsAccordingToJointLimits()
      {
         ReferenceFrame frameBeforeHipYawJoint = hipYawJoint.getFrameBeforeJoint();
         desiredOrientation.changeFrame(frameBeforeHipYawJoint);
         desiredOrientation.getYawPitchRoll(desiredYawPitchRoll);
         if (desiredYawPitchRoll[0] > hipYawJoint.getJointLimitUpper() - epsilon)
         {
            desiredYawPitchRoll[0] = hipYawJoint.getJointLimitUpper();
            desiredAngularVelocity.changeFrame(frameBeforeHipYawJoint);
            desiredAngularVelocity.setZ(Math.min(0.0, desiredAngularVelocity.getZ()));
            desiredAngularAcceleration.changeFrame(frameBeforeHipYawJoint);
            desiredAngularAcceleration.setZ(Math.min(0.0, desiredAngularVelocity.getZ()));
         }
         else if (desiredYawPitchRoll[0] < hipYawJoint.getJointLimitLower() + epsilon)
         {
            desiredYawPitchRoll[0] = hipYawJoint.getJointLimitLower();
            desiredAngularVelocity.changeFrame(frameBeforeHipYawJoint);
            desiredAngularVelocity.setZ(Math.max(0.0, desiredAngularVelocity.getZ()));
            desiredAngularAcceleration.changeFrame(frameBeforeHipYawJoint);
            desiredAngularAcceleration.setZ(Math.max(0.0, desiredAngularVelocity.getZ()));
         }
         desiredOrientation.setYawPitchRoll(desiredYawPitchRoll);

         ReferenceFrame frameBeforeAnklePitchJoint = anklePitchJoint.getFrameBeforeJoint();
         desiredOrientation.changeFrame(frameBeforeAnklePitchJoint);
         desiredOrientation.getYawPitchRoll(desiredYawPitchRoll);
         if (desiredYawPitchRoll[1] > anklePitchJoint.getJointLimitUpper() - epsilon)
         {
            desiredYawPitchRoll[1] = anklePitchJoint.getJointLimitUpper();
            desiredAngularVelocity.changeFrame(frameBeforeAnklePitchJoint);
            desiredAngularVelocity.setY(Math.min(0.0, desiredAngularVelocity.getY()));
            desiredAngularAcceleration.changeFrame(frameBeforeAnklePitchJoint);
            desiredAngularAcceleration.setY(Math.min(0.0, desiredAngularVelocity.getY()));
         }
         else if (desiredYawPitchRoll[1] < anklePitchJoint.getJointLimitLower() + epsilon)
         {
            desiredYawPitchRoll[1] = anklePitchJoint.getJointLimitLower();
            desiredAngularVelocity.changeFrame(frameBeforeAnklePitchJoint);
            desiredAngularVelocity.setY(Math.max(0.0, desiredAngularVelocity.getY()));
            desiredAngularAcceleration.changeFrame(frameBeforeAnklePitchJoint);
            desiredAngularAcceleration.setY(Math.max(0.0, desiredAngularVelocity.getY()));
         }
         desiredOrientation.setYawPitchRoll(desiredYawPitchRoll);

         ReferenceFrame frameBeforeAnkleRollJoint = ankleRollJoint.getFrameBeforeJoint();
         desiredOrientation.changeFrame(frameBeforeAnkleRollJoint);
         desiredOrientation.getYawPitchRoll(desiredYawPitchRoll);
         if (desiredYawPitchRoll[2] > ankleRollJoint.getJointLimitUpper() - epsilon)
         {
            desiredYawPitchRoll[2] = ankleRollJoint.getJointLimitUpper();
            desiredAngularVelocity.changeFrame(frameBeforeAnkleRollJoint);
            desiredAngularVelocity.setX(Math.min(0.0, desiredAngularVelocity.getX()));
            desiredAngularAcceleration.changeFrame(frameBeforeAnkleRollJoint);
            desiredAngularAcceleration.setX(Math.min(0.0, desiredAngularVelocity.getX()));
         }
         else if (desiredYawPitchRoll[2] < ankleRollJoint.getJointLimitLower() + epsilon)
         {
            desiredYawPitchRoll[2] = ankleRollJoint.getJointLimitLower();
            desiredAngularVelocity.changeFrame(frameBeforeAnkleRollJoint);
            desiredAngularVelocity.setX(Math.max(0.0, desiredAngularVelocity.getX()));
            desiredAngularAcceleration.changeFrame(frameBeforeAnkleRollJoint);
            desiredAngularAcceleration.setX(Math.max(0.0, desiredAngularVelocity.getX()));
         }
         desiredOrientation.setYawPitchRoll(desiredYawPitchRoll);

         desiredOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.changeFrame(worldFrame);
         desiredAngularAcceleration.changeFrame(worldFrame);
      }

      private final void updateVisualizers()
      {
         if (!visualize)
            return;

         desiredOrientationFrame.update();
         correctedDesiredOrientationFrame.update();
         desiredOrientationFrameGraphic.update();
         correctedDesiredOrientationFrameGraphic.update();
      }

      public void doTransitionOutOfAction()
      {
         yoDesiredPosition.setToNaN();
         trajectoryWasReplanned.set(false);
         isUnconstrained.set(false);

         accelerationControlModule.reset();

         if (visualize)
         {
            desiredOrientationFrameGraphic.hideGraphicObject();
            correctedDesiredOrientationFrameGraphic.hideGraphicObject();
         }
      }
   }

   private class FootStateTransitionCondition implements StateTransitionCondition
   {
      private final ConstraintType stateEnum;

      public FootStateTransitionCondition(FootControlState stateToTransitionTo)
      {
         this.stateEnum = stateToTransitionTo.getStateEnum();
      }

      public boolean checkCondition()
      {
         boolean transitionRequested = requestedState.getEnumValue() == stateEnum;

         if (!transitionRequested)
            return false;
         
         boolean isTransitionToCurrentState = requestedState.getEnumValue() == getCurrentConstraintType();
         
         if (isTransitionToCurrentState)
            return false;

         boolean singularityEscapeDone = true;

         if (doSingularityEscape.getBooleanValue() && waitSingularityEscapeBeforeTransitionToNextState.getBooleanValue())
         {
            nullspaceCalculator.setMatrix(jacobian.getJacobianMatrix(), 1);
            DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();
            ScrewTools.packJointVelocitiesMatrix(jacobian.getJointsInOrder(), jointVelocities);
            double nullspaceVelocityDotProduct = VectorVectorMult.innerProd(nullspace, jointVelocities);

            int velocitySign = (int) Math.round(Math.signum(nullspaceVelocityDotProduct));
            boolean velocitySignOK = velocitySign == velocitySignForSingularityEscape;
            if (jacobianDeterminantInRange.getBooleanValue() || !velocitySignOK)
            {
               singularityEscapeDone = false;
            }
         }

         return singularityEscapeDone;
      }
   }

   public class FootStateTransitionAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         requestedState.set(null);
         doSingularityEscape.set(false);
         waitSingularityEscapeBeforeTransitionToNextState.set(false);
      }
   }

   // TODO: don't hardcode this here:
   private void setSwingControlGains(double kxyPosition, double kzPosition, double kOrientation, double zetaXYZ, double zetaOrientation)
   {
      double dxyPosition = GainCalculator.computeDerivativeGain(kxyPosition, zetaXYZ);
      double dzPosition = GainCalculator.computeDerivativeGain(kzPosition, zetaXYZ);
      double dOrientation = GainCalculator.computeDerivativeGain(kOrientation, zetaOrientation);

      footSpatialAccelerationControlModule.setPositionProportionalGains(kxyPosition, kxyPosition, kzPosition);
      footSpatialAccelerationControlModule.setPositionDerivativeGains(dxyPosition, dxyPosition, dzPosition);
      footSpatialAccelerationControlModule.setOrientationProportionalGains(kOrientation, kOrientation, kOrientation);
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(dOrientation, dOrientation, dOrientation);
   }

   // TODO Should we maintain the pitch and roll?
   private final void setHoldPositionStateGains()
   {
      double dxPosition = GainCalculator.computeDerivativeGain(holdKpx.getDoubleValue(), holdZeta.getDoubleValue());
      double dyPosition = GainCalculator.computeDerivativeGain(holdKpy.getDoubleValue(), holdZeta.getDoubleValue());
      double dzPosition = GainCalculator.computeDerivativeGain(holdKpz.getDoubleValue(), holdZeta.getDoubleValue());
      double dxOrientation = GainCalculator.computeDerivativeGain(holdKpRoll.getDoubleValue(), holdZeta.getDoubleValue());
      double dyOrientation = GainCalculator.computeDerivativeGain(holdKpPitch.getDoubleValue(), holdZeta.getDoubleValue());
      double dzOrientation = GainCalculator.computeDerivativeGain(holdKpYaw.getDoubleValue(), holdZeta.getDoubleValue());

      footSpatialAccelerationControlModule.setPositionProportionalGains(holdKpx.getDoubleValue(), holdKpy.getDoubleValue(), holdKpz.getDoubleValue());
      footSpatialAccelerationControlModule.setPositionDerivativeGains(dxPosition, dyPosition, dzPosition);
      footSpatialAccelerationControlModule.setOrientationProportionalGains(holdKpRoll.getDoubleValue(), holdKpPitch.getDoubleValue(), holdKpYaw.getDoubleValue());
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(dxOrientation, dyOrientation, dzOrientation);
   }

   //TODO: Get rid of these set gain methods. Pull them up or something.
   private void setStanceControlGains()
   {
      // position gains need to be zero right now, because position trajectory doesn't match orientation trajectory. projection onto motion subspace will fix inconsistent acceleration.
      setGains(0.0, 500.0, 1.0);
   }

   private void setOnToesFreeMotionGains()
   {
      double kPosition = 0.0;
      double dPosition = GainCalculator.computeDerivativeGain(0.0, toeOffZeta.getDoubleValue());
      double dxOrientation = GainCalculator.computeDerivativeGain(toeOffKpRoll.getDoubleValue(), toeOffZeta.getDoubleValue());
      double dyOrientation = GainCalculator.computeDerivativeGain(toeOffKpPitch.getDoubleValue(), toeOffZeta.getDoubleValue());
      double dzOrientation = GainCalculator.computeDerivativeGain(toeOffKpYaw.getDoubleValue(), toeOffZeta.getDoubleValue());

      footSpatialAccelerationControlModule.setPositionProportionalGains(kPosition, kPosition, kPosition);
      footSpatialAccelerationControlModule.setPositionDerivativeGains(dPosition, dPosition, dPosition);
      footSpatialAccelerationControlModule.setOrientationProportionalGains(toeOffKpRoll.getDoubleValue(), toeOffKpPitch.getDoubleValue(), toeOffKpYaw.getDoubleValue());
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(dxOrientation, dyOrientation, dzOrientation);
   }

   private void setTouchdownOnEdgeGains()
   {
      double kPosition = 0.0;
      double dPosition = GainCalculator.computeDerivativeGain(kPosition, 1.0);
      double kxzOrientation = 300.0;
      double kyOrientation = 0.0;
      double dxzOrientation = GainCalculator.computeDerivativeGain(kxzOrientation, 0.4);
      double dyOrientation = GainCalculator.computeDerivativeGain(kxzOrientation, 0.4); //Only damp the pitch velocity

      footSpatialAccelerationControlModule.setPositionProportionalGains(kPosition, kPosition, kPosition);
      footSpatialAccelerationControlModule.setPositionDerivativeGains(dPosition, dPosition, dPosition);
      footSpatialAccelerationControlModule.setOrientationProportionalGains(kxzOrientation, kyOrientation, kxzOrientation);
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(dxzOrientation, dyOrientation, dxzOrientation);
   }

   private void setGains(double kPosition, double kOrientation, double zeta)
   {
      double dPosition = GainCalculator.computeDerivativeGain(kPosition, zeta);
      double dOrientation = GainCalculator.computeDerivativeGain(kOrientation, zeta);
      
      footSpatialAccelerationControlModule.setPositionProportionalGains(kPosition, kPosition, kPosition);
      footSpatialAccelerationControlModule.setPositionDerivativeGains(dPosition, dPosition, dPosition);
      footSpatialAccelerationControlModule.setOrientationProportionalGains(kOrientation, kOrientation, kOrientation);
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(dOrientation, dOrientation, dOrientation);
   }

   private final FrameConvexPolygon2d contactPolygon = new FrameConvexPolygon2d();
   private final FrameOrientation currentOrientation = new FrameOrientation();
   private void determineCoPOnEdge()
   {
      FramePoint2d cop = momentumBasedController.getCoP(contactableBody);

      if (cop == null)
      {
         isCoPOnEdge.set(false);
      }
      else
      {
         List<FramePoint2d> contactPoints = contactableBody.getContactPoints2d();
         contactPolygon.setIncludingFrameAndUpdate(contactPoints);
         cop.changeFrame(contactPolygon.getReferenceFrame());
         FrameLineSegment2d closestEdge = contactPolygon.getClosestEdge(cop);
         boolean copOnEdge = closestEdge.distance(cop) < epsilonPointOnEdge;
         boolean hasCoPBeenOnEdge = isCoPOnEdge.getBooleanValue();
         if (copOnEdge && !hasCoPBeenOnEdge)
         {
            currentOrientation.set(getFootFrame());
            currentOrientation.changeFrame(worldFrame);
            orientationFix.set(currentOrientation);
         }
         isCoPOnEdge.set(copOnEdge);

         this.edgeToRotateAbout.setFrameLineSegment2d(closestEdge);
      }
   }

   public ReferenceFrame getFootFrame()
   {
      return contactableBody.getBodyFrame();
   }

   public void doControl()
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.update();
      computeNullspaceMultipliers();
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      
      coefficientOfFrictionFiltered.update();
      momentumBasedController.setPlaneContactCoefficientOfFriction(contactableBody, coefficientOfFrictionFiltered.getDoubleValue());
   }
   
   private void computeNullspaceMultipliers()
   {
      double det = jacobian.det();
      jacobianDeterminant.set(det);
      jacobianDeterminantInRange.set(Math.abs(det) < minJacobianDeterminant.getDoubleValue());

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

   private void setTaskspaceConstraint(SpatialAccelerationVector footAcceleration)
   {
      ReferenceFrame bodyFixedFrame = contactableBody.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      taskspaceConstraintData.set(footAcceleration, nullspaceMultipliers, selectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
   }

   public boolean isTrajectoryDone()
   {
      return isTrajectoryDone.getBooleanValue();
   }

   public void resetTrajectoryDone()
   {
      isTrajectoryDone.set(false);
   }

   // Used to restart the current state reseting the current state time
   public void resetCurrentState()
   {
      stateMachine.setCurrentState(getCurrentConstraintType());
   }

   public boolean isInFlatSupportState()
   {
      return getCurrentConstraintType() == ConstraintType.FULL || getCurrentConstraintType() == ConstraintType.HOLD_POSITION;
   }
   
   public boolean isInEdgeTouchdownState()
   {
      return getCurrentConstraintType() == ConstraintType.HEEL_TOUCHDOWN || getCurrentConstraintType() == ConstraintType.TOES_TOUCHDOWN;
   }

   private List<FramePoint2d> getEdgeContactPoints2d(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector direction = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      if (constraintType == ConstraintType.HEEL_TOUCHDOWN)
         direction.scale(-1.0);

      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPointsCopy(), direction,
            NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT);

      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }

   private boolean[] getOnEdgeContactPointStates(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector direction = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      if (constraintType == ConstraintType.HEEL_TOUCHDOWN)
         direction.scale(-1.0);

      int[] indexOfPointsInContact = DesiredFootstepCalculatorTools.findMaximumPointIndexesInDirection(contactableBody.getContactPointsCopy(), direction, 2);

      boolean[] contactPointStates = new boolean[contactableBody.getTotalNumberOfContactPoints()];

      for (int i = 0; i < indexOfPointsInContact.length; i++)
      {
         contactPointStates[indexOfPointsInContact[i]] = true;
      }

      return contactPointStates;
   }

   public double updateAndGetLegLength()
   {
      return legSingularityAndKneeCollapseAvoidanceControlModule.updateAndGetLegLength();
   }

   public void correctCoMHeightTrajectoryForSingularityAvoidance(FrameVector2d comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect, double zCurrent,
         ReferenceFrame pelvisZUpFrame)
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent, pelvisZUpFrame, getCurrentConstraintType());
   }

   public void correctCoMHeightTrajectoryForCollapseAvoidance(FrameVector2d comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect, double zCurrent,
         ReferenceFrame pelvisZUpFrame, double footLoadPercentage)
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForCollapseAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent, pelvisZUpFrame, footLoadPercentage, getCurrentConstraintType());
   }
   
   public void correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesData comHeightDataToCorrect)
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForUnreachableFootStep(comHeightDataToCorrect, getCurrentConstraintType());
   }
}