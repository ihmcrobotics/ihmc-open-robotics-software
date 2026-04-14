package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingPostContactState;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingPreContactState;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingState;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingStateEnum;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.DeadbandTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RigidBodyDynamicLoadBearingControlState extends RigidBodyControlState
{
   private static final FrameVector3D zeroWorld = new FrameVector3D();

   private static final double CAPTURE_POINT_ERROR_THRESHOLD_TO_REMAIN_IN_STATE = 0.02;
   private static final double CAPTURE_POINT_DISTANCE_INSIDE_NOMINAL_SUPPORT_THRESHOLD = 0.04;

   private final StateMachine<DynamicLoadBearingStateEnum, DynamicLoadBearingState> stateMachine;
   private final DynamicLoadBearingPreContactState preContactState;
   private final DynamicLoadBearingPostContactState postContactState;
   private final RigidBodyBasics bodyToControl;
   private final LoadBearingParameters loadBearingParameters;
   private final ReferenceFrame controlFrame;

   private final RigidBodyJointControlHelper jointControlHelper;
   private final Runnable onExitRunnable;
   private final DoubleProvider capturePointErrorProvider;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final FramePoint3DReadOnly capturePoint;
   private final Plane3D bracingPlane = new Plane3D();

   private final FramePoint3D handPosition = new FramePoint3D();
   private final FramePoint3D elbowPosition = new FramePoint3D();
   private final FrameVector3D handToElbowVector = new FrameVector3D();

   /* Elbow collisions */
   private final PointFeedbackControlCommand collisionAvoidanceCommand = new PointFeedbackControlCommand();
   private final RigidBodyBasics bodyToAvoidCollisions;
   private final FramePoint3D collisionAvoidancePointInBody = new FramePoint3D();
   private final FramePoint3D desiredCollisionAvoidancePointInWorld = new FramePoint3D();
   private final FrameVector3D deltaCollisionDistance = new FrameVector3D();
   private final YoBoolean isCollisionAvoidanceActivated;
   private final YoDouble handToElbowDistance;
   private final YoDouble alphaCollisionActivation;
   private final Vector3D collisionWeight = new Vector3D();
   private final Vector3D defaultCollisionWeight = new Vector3D(0.0, 0.0, 3.5);

   private final OneDoFJointBasics shoulderXJoint;
   private final double desiredJointPositionForCollisionAvoidance;
   private final OneDoFJointFeedbackControlCommand jointFeedbackControlCommand = new OneDoFJointFeedbackControlCommand();

   private final FramePose3D bracingPose = new FramePose3D();
   private final PoseReferenceFrame bracingFrame;

   public RigidBodyDynamicLoadBearingControlState(RigidBodyBasics bodyToControl,
                                                  RigidBodyBasics baseBody,
                                                  RigidBodyBasics elevator,
                                                  LoadBearingParameters loadBearingParameters,
                                                  YoDouble yoTime,
                                                  Runnable onExitRunnable,
                                                  RigidBodyJointControlHelper jointControlHelper,
                                                  RigidBodyPositionControlHelper positionControlHelper,
                                                  RigidBodyOrientationControlHelper orientationControlHelper,
                                                  ReferenceFrame controlFrame,
                                                  double nominalRhoWeight,
                                                  DoubleProvider capturePointErrorProvider,
                                                  MutableBoolean hasAddedContacts,
                                                  MutableBoolean hasRemovedContacts,
                                                  BipedSupportPolygons bipedSupportPolygons,
                                                  FramePoint3DReadOnly capturePoint,
                                                  YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.DYNAMIC_LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Bracing";

      this.controlFrame = controlFrame;
      this.loadBearingParameters = loadBearingParameters;
      this.bodyToControl = bodyToControl;

      preContactState = new DynamicLoadBearingPreContactState(bodyToControl, positionControlHelper, controlFrame, loadBearingParameters, bracingPlane, registry);
      postContactState = new DynamicLoadBearingPostContactState(bodyToControl,
                                                                baseBody,
                                                                elevator,
                                                                controlFrame,
                                                                positionControlHelper,
                                                                orientationControlHelper,
                                                                loadBearingParameters,
                                                                nominalRhoWeight,
                                                                hasAddedContacts,
                                                                hasRemovedContacts,
                                                                bracingPlane,
                                                                registry);

      stateMachine = setupStateMachine(namePrefix, yoTime);

      this.onExitRunnable = onExitRunnable;
      this.jointControlHelper = jointControlHelper;
      this.capturePointErrorProvider = capturePointErrorProvider;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.capturePoint = capturePoint;

      bracingFrame = new PoseReferenceFrame("desiredContactFrame" + bodyName, ReferenceFrame.getWorldFrame());
      alphaCollisionActivation = new YoDouble(bodyName + "_alphaCollision", registry);
      handToElbowDistance = new YoDouble(bodyName + "_CollisionDistance", registry);

      // Collision avoidance
      bodyToAvoidCollisions = bodyToControl.getParentJoint().getPredecessor();
      collisionAvoidancePointInBody.setToZero(bodyToControl.getParentJoint().getFrameAfterJoint());
      collisionAvoidancePointInBody.changeFrame(bodyToAvoidCollisions.getBodyFixedFrame());
      isCollisionAvoidanceActivated = new YoBoolean("isCollisionAvoidanceActivated", registry);
      collisionAvoidanceCommand.set(elevator, bodyToAvoidCollisions);
      collisionAvoidanceCommand.setPrimaryBase(baseBody);
      collisionAvoidanceCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      String prefix = bodyToControl.getParentJoint().getName().split("_")[0];
      RobotSide robotSide = prefix.contains("LEFT") ? RobotSide.LEFT : RobotSide.RIGHT;
      shoulderXJoint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(baseBody, prefix + "_SHOULDER_X");
      jointFeedbackControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      jointFeedbackControlCommand.setJoint(shoulderXJoint);
      desiredJointPositionForCollisionAvoidance = robotSide.negateIfRightSide(0.05);
      jointFeedbackControlCommand.getGains().set(100.0, 3.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   }

   private StateMachine<DynamicLoadBearingStateEnum, DynamicLoadBearingState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<DynamicLoadBearingStateEnum, DynamicLoadBearingState> factory = new StateMachineFactory<>(DynamicLoadBearingStateEnum.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(DynamicLoadBearingStateEnum.PRE_CONTACT, preContactState);
      factory.addState(DynamicLoadBearingStateEnum.POST_CONTACT, postContactState);
      factory.addDoneTransition(DynamicLoadBearingStateEnum.PRE_CONTACT, DynamicLoadBearingStateEnum.POST_CONTACT);
//      factory.addDoneTransition(DynamicLoadBearingStateEnum.POST_CONTACT, DynamicLoadBearingStateEnum.PRE_CONTACT);

      return factory.build(DynamicLoadBearingStateEnum.PRE_CONTACT);
   }

   public void setBracingSurface(HandContactCommand command)
   {
      bracingPlane.set(command.getBracingPoint(), command.getBracingNormal());
      preContactState.setBracingData(command);
      postContactState.setBracingSurface(command.getBracingNormal());

      bracingPose.setToZero(ReferenceFrame.getWorldFrame());
      bracingPose.getPosition().setMatchingFrame(command.getBracingPoint());
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, command.getBracingNormal(), bracingPose.getOrientation());
      bracingFrame.setPoseAndUpdate(bracingPose);
   }

   @Override
   public void onEntry()
   {
      stateMachine.performTransition(DynamicLoadBearingStateEnum.PRE_CONTACT, false, true);
   }

   @Override
   public void doAction(double timeInState)
   {
      // Do state-specific action
      stateMachine.doActionAndTransition();

      // Check for potential elbow collision
      handPosition.setToZero(controlFrame);
      handPosition.changeFrame(ReferenceFrame.getWorldFrame());
      elbowPosition.setToZero(bodyToControl.getParentJoint().getFrameAfterJoint());
      elbowPosition.changeFrame(ReferenceFrame.getWorldFrame());
      handToElbowVector.sub(elbowPosition, handPosition);
      handToElbowDistance.set(handToElbowVector.dot(bracingPlane.getNormal()));

      double maxActivationDistance = 0.18;
      double minActivationDistance = 0.09;
      boolean isCollisionAvoidanceActivated = handToElbowDistance.getValue() < maxActivationDistance;
      alphaCollisionActivation.set(1.0 - (handToElbowDistance.getValue() - minActivationDistance) / (maxActivationDistance - minActivationDistance));
      alphaCollisionActivation.set(EuclidCoreTools.clamp(alphaCollisionActivation.getValue(), 0.0, 1.0));

      if (alphaCollisionActivation.getValue() > 0.0)
      { // compute setpoint
         collisionAvoidanceCommand.setBodyFixedPointToControl(collisionAvoidancePointInBody);
         collisionAvoidanceCommand.setGainsFrame(bracingFrame);

         desiredCollisionAvoidancePointInWorld.setMatchingFrame(collisionAvoidancePointInBody);
         deltaCollisionDistance.set(bracingPlane.getNormal());
         deltaCollisionDistance.scale(alphaCollisionActivation.getValue() * (maxActivationDistance - minActivationDistance));
         desiredCollisionAvoidancePointInWorld.add(deltaCollisionDistance);

         collisionAvoidanceCommand.setInverseDynamics(desiredCollisionAvoidancePointInWorld, zeroWorld, zeroWorld);
         collisionAvoidanceCommand.setGains(loadBearingParameters.getCollisionGains());

         collisionWeight.set(defaultCollisionWeight);
         collisionWeight.scale(Math.max(alphaCollisionActivation.getValue(), 0.5));
         collisionAvoidanceCommand.setWeightsForSolver(collisionWeight);
      }

      double minWeight = 0.1;
      double maxWeight = 2.5;
      jointFeedbackControlCommand.setWeightForSolver(Math.max(minWeight, alphaCollisionActivation.getValue() * maxWeight));

      double currentQ = shoulderXJoint.getQ();
      double maxDelta = Math.toRadians(30.0);
      double desiredQ = currentQ + EuclidCoreTools.clamp(desiredJointPositionForCollisionAvoidance - currentQ, maxDelta);
      jointFeedbackControlCommand.setInverseDynamics(desiredQ, 0.0, 0.0);

      this.isCollisionAvoidanceActivated.set(isCollisionAvoidanceActivated);
   }

   @Override
   public void onExit(double timeInState)
   {
      onExitRunnable.run();
      postContactState.onExit(0.0);
   }

   private final FramePoint2D tempPoint = new FramePoint2D();

   @Override
   public boolean isDone(double timeInState)
   {
      boolean hasLowTrackingError = capturePointErrorProvider.getValue() < CAPTURE_POINT_ERROR_THRESHOLD_TO_REMAIN_IN_STATE;
      if (hasLowTrackingError)
         return true;

      boolean doCapturePointInSupportCheck = stateMachine.getCurrentStateKey() == DynamicLoadBearingStateEnum.POST_CONTACT || stateMachine.getTimeInCurrentState() > 0.8 * preContactState.getTrajectoryDuration();
      if (doCapturePointInSupportCheck)
      {
         tempPoint.setIncludingFrame(capturePoint);
         boolean isCapturePointInSupportPolygon = bipedSupportPolygons.getSupportPolygonInWorld().signedDistance(tempPoint) < -CAPTURE_POINT_DISTANCE_INSIDE_NOMINAL_SUPPORT_THRESHOLD;
         if (isCapturePointInSupportPolygon)
            return true;
      }

      if (stateMachine.getCurrentStateKey() == DynamicLoadBearingStateEnum.PRE_CONTACT)
      {
         // Only return if the arm is stuck, which is likely because it's extended
         return preContactState.isStuck(stateMachine.getTimeInCurrentState());
      }
      else
      {
         // If the hand is slipping or the arm is straightened, exit this state
         boolean isSlippingOrAtSingularity = stateMachine.getCurrentState().isDone(stateMachine.getTimeInCurrentState());
         if (isSlippingOrAtSingularity)
            return true;

         return false;
      }
   }

   @Override
   public boolean isEmpty()
   {
      // this control mode does not support command queuing
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      // this control mode does not support command queuing
      return 0.0;
   }

   public void setOnTouchdownCallback(Runnable runnable)
   {
      postContactState.setOnTouchdownCallback(runnable);
   }

   public boolean isLoadBearing()
   {
      return stateMachine.getCurrentStateKey() == DynamicLoadBearingStateEnum.POST_CONTACT;
   }

   public void updateWholeBodyContactState(WholeBodyContactState wholeBodyContactStateToUpdate)
   {
      if (isLoadBearing())
         postContactState.updateWholeBodyContactState(wholeBodyContactStateToUpdate);
   }

   public void packContactData(RecyclingArrayList<Point3D> contactPointList, Vector3DBasics contactNormalToPack)
   {
      if (isLoadBearing())
      {
         postContactState.packContactData(contactPointList, contactNormalToPack);
      }
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      postContactState.setControllerCoreOutput(controllerCoreOutput);
   }

   @Override
   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      return stateMachine.getCurrentState().getTransitionOutOfStateCommand();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(preContactState.getSCS2YoGraphics());
      group.addChild(postContactState.getSCS2YoGraphics());
      return group;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();

      feedbackControlCommandList.addCommand(stateMachine.getCurrentState().getFeedbackControlCommand());
      if (loadBearingParameters.enableCollisionAvoidance())
         feedbackControlCommandList.addCommand(jointFeedbackControlCommand);

      return feedbackControlCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

//      feedbackControlCommandList.addCommand(collisionAvoidanceCommand);
      feedbackControlCommandList.addCommand(jointFeedbackControlCommand);

      for (DynamicLoadBearingStateEnum mode : DynamicLoadBearingStateEnum.values())
      {
         DynamicLoadBearingState state = stateMachine.getState(mode);
         if (state != null && state.createFeedbackControlTemplate() != null)
            feedbackControlCommandList.addCommand(state.createFeedbackControlTemplate());
      }
      return feedbackControlCommandList;
   }
}
