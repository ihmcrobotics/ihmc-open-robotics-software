package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.LoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * A load bearing state for a rigid body. This is currently for using the hands
 * to help support the weight of the robot. It is based on a contact model
 * of a single point on the hand contacting an environmental plane.
 */
public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int dofs = Twist.SIZE;
   private static final FrameVector3D zeroInWorld = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);

   private final RigidBodyBasics bodyToControl;
   private final LoadBearingParameters loadBearingParameters;
   private final SelectionMatrix6D spatialFeedbackSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D spatialAccelerationSelectionMatrix = new SelectionMatrix6D();

   /* Controller Commands */
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   /* Control gains, weights and axis selection */
   private final YoBoolean[] isAngularAxisFeedbackControlled = new YoBoolean[3];
   private final YoBoolean[] isLinearAxisFeedbackControlled = new YoBoolean[3];
   private final PIDSE3GainsReadOnly holdHandPositionGains;
   private final Vector3DReadOnly handLoadedLinearWeight = new Vector3D(5.0, 5.0, 5.0);
   private final Vector3DReadOnly handLoadedAngularWeight = new Vector3D(0.2, 0.2, 0.2);
   private final YoEnum<LoadBearingControlMode> controlMode;

   /* Hand load status */
   private final YoBoolean bodyBarelyLoaded;
   private ControllerCoreOutputReadOnly controllerCoreOutput;
   private final Wrench controllerDesiredWrench = new Wrench();
   private final FrameVector3D controllerDesiredForce = new FrameVector3D();
   private final YoFrameVector3D yoControllerDesiredForce;

   /* Reference frames */
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final FramePose3D contactPoseInBodyFrame = new FramePose3D();
   private final FramePose3D contactPoseInWorldFrame = new FramePose3D();
   private final PoseReferenceFrame contactFrame;

   /* Yo-Contact frame components */
   private final YoFrameVector3D contactNormal;
   private final YoFramePoint3D currentContactPointInWorld;
   private final YoFrameQuaternion currentContactOrientationInWorld;
   private final YoFramePoint3D desiredContactPointInWorld;
   private final YoFrameQuaternion desiredContactOrientationInWorld;
   private final YoFramePoint3D contactPoint;

   /* Trajectory handlers */
   private final RigidBodyJointControlHelper jointControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;

   private final YoDouble coefficientOfFriction;
   private final SpatialAcceleration bodyAcceleration;

   /* Error measurements */
   private final YoFramePoint3D positionError;
   private final YoFrameQuaternion orientationError;

   public RigidBodyLoadBearingControlState(RigidBodyBasics bodyToControl,
                                           RigidBodyBasics elevator,
                                           YoDouble yoTime,
                                           RigidBodyJointControlHelper jointControlHelper,
                                           RigidBodyOrientationControlHelper orientationControlHelper,
                                           YoGraphicsListRegistry graphicsListRegistry,
                                           YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.bodyToControl = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();

      String bodyName = bodyToControl.getName();

      contactFrame = new PoseReferenceFrame("contactFrame" + bodyName, bodyFrame);
      bodyAcceleration = new SpatialAcceleration(contactFrame, elevatorFrame, contactFrame);
      spatialFeedbackControlCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.set(elevator, bodyToControl);
      contactPoint = new YoFramePoint3D("contactPoint" + bodyName, contactFrame, registry);

      // TODO move this up so left/right have same gains
      this.holdHandPositionGains = new ParameterizedPIDSE3Gains("Hold" + bodyToControl.getName(), getHoldPositionHandControlGains(), registry);

      coefficientOfFriction = new YoDouble(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector3D(bodyName + "ContactNormal", worldFrame, parentRegistry);
      currentContactPointInWorld = new YoFramePoint3D(bodyName + "currentContactPoint", worldFrame, registry);
      currentContactOrientationInWorld = new YoFrameQuaternion(bodyName + "currentContactOrientation", worldFrame, registry);
      desiredContactPointInWorld = new YoFramePoint3D(bodyName + "desiredContactPoint", worldFrame, registry);
      desiredContactOrientationInWorld = new YoFrameQuaternion(bodyName + "desiredContactOrientation", worldFrame, registry);

      controlMode = new YoEnum<>(bodyName + "ControlMode", parentRegistry, LoadBearingControlMode.class);
      bodyBarelyLoaded = new YoBoolean(bodyName + "BarelyLoaded", registry);
      yoControllerDesiredForce = new YoFrameVector3D(bodyName + "DesiredForce", ReferenceFrame.getWorldFrame(), parentRegistry);

      positionError = new YoFramePoint3D(bodyName + "PositionError", contactFrame, registry);
      orientationError = new YoFrameQuaternion(bodyName + "OrientationError", contactFrame, registry);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      this.loadBearingParameters = new LoadBearingParameters(bodyName, registry);
      this.jointControlHelper = jointControlHelper;
      this.orientationControlHelper = orientationControlHelper;

      for (int i = 0; i < 3; i++)
      {
         isAngularAxisFeedbackControlled[i] = new YoBoolean("isAngular" + Axis3D.values[i] + "FeedbackControlled", registry);
         isLinearAxisFeedbackControlled[i] = new YoBoolean("isLinear" + Axis3D.values[i] + "FeedbackControlled", registry);
      }

      setupViz(graphicsListRegistry, bodyName);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry, String bodyName)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();

      YoGraphicVector surfaceNormal = new YoGraphicVector(bodyName + "ContactNormal", currentContactPointInWorld, contactNormal, 0.1, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, surfaceNormal);
      graphics.add(surfaceNormal);

      YoGraphicPosition contactPoint = new YoGraphicPosition(bodyName + "ContactPoint", currentContactPointInWorld, 0.01, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, contactPoint);
      graphics.add(contactPoint);

      YoGraphicVector controllerDesiredForce = new YoGraphicVector(bodyName + "DesiredForceGraphic",
                                                                   currentContactPointInWorld,
                                                                   yoControllerDesiredForce,
                                                                   0.015,
                                                                   YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic(listName, controllerDesiredForce);
      graphics.add(controllerDesiredForce);

      YoGraphicCoordinateSystem controlFrame = new YoGraphicCoordinateSystem(bodyName + "LoadBearingControlFrame", currentContactPointInWorld,
                                                                             currentContactOrientationInWorld, 0.2, YoAppearance.DarkGray());
      graphicsListRegistry.registerYoGraphic(listName, controlFrame);
      graphics.add(controlFrame);

      YoGraphicCoordinateSystem desiredFrame = new YoGraphicCoordinateSystem(bodyName + "LoadBearingDesiredFrame", desiredContactPointInWorld,
                                                                             desiredContactOrientationInWorld, 0.17, YoAppearance.LightGray());
      graphicsListRegistry.registerYoGraphic(listName, desiredFrame);
      graphics.add(controlFrame);

      hideGraphics();
   }

   @Override
   public void doAction(double timeInState)
   {
      // Update contact frame/point
      contactFrame.update();
      contactPoint.setToZero();

      // Update YoVariables for contact visualization
      currentContactPointInWorld.setMatchingFrame(contactPoseInBodyFrame.getPosition());
      currentContactOrientationInWorld.setMatchingFrame(contactPoseInBodyFrame.getOrientation());
      desiredContactPointInWorld.setMatchingFrame(contactPoseInWorldFrame.getPosition());
      desiredContactOrientationInWorld.setMatchingFrame(contactPoseInWorldFrame.getOrientation());

      if (controllerCoreOutput.getDesiredExternalWrench(controllerDesiredWrench, bodyToControl))
      { // Determine load status from controller core desired, assume it tracks
         double desiredForceLoadMagnitudeSquared = controllerDesiredWrench.getLinearPart().normSquared();
         bodyBarelyLoaded.set(desiredForceLoadMagnitudeSquared < MathTools.square(loadBearingParameters.getNormalForceThresholdForLoaded()));

         controllerDesiredForce.setIncludingFrame(controllerDesiredWrench.getReferenceFrame(), controllerDesiredWrench.getLinearPart());
         controllerDesiredForce.changeFrame(ReferenceFrame.getWorldFrame());
         yoControllerDesiredForce.set(controllerDesiredForce);
      }
      else
      { // If no desired wrench, set to barely loaded
         bodyBarelyLoaded.set(true);
      }

      // assemble contact command
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal);
      planeContactStateCommand.addPointInContact(getContactPoint());
      planeContactStateCommand.setHasContactStateChanged(false);

      positionError.setMatchingFrame(desiredContactPointInWorld);
      orientationError.setMatchingFrame(desiredContactOrientationInWorld);

      // assemble spatial feedback command
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(contactPoseInBodyFrame);
      //      spatialFeedbackControlCommand.setInverseDynamics(contactFrameInWorldFrame.getOrientation(), contactFrameInWorldFrame.getPosition(), zeroInWorld, zeroInWorld, zeroInWorld, zeroInWorld);

      zeroInWorld.setToZero(contactFrame);
      spatialFeedbackControlCommand.setInverseDynamics(orientationError, positionError, zeroInWorld, zeroInWorld, zeroInWorld, zeroInWorld);
      spatialFeedbackControlCommand.setWeightsForSolver(handLoadedAngularWeight, handLoadedLinearWeight);
      spatialFeedbackControlCommand.setOrientationGains(holdHandPositionGains.getOrientationGains());
      spatialFeedbackControlCommand.setPositionGains(holdHandPositionGains.getPositionGains());

      // assemble zero acceleration command
      bodyAcceleration.setToZero(contactFrame, elevatorFrame, contactFrame);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(contactFrame, bodyAcceleration);

      // set axis control strategies
      isLinearAxisFeedbackControlled[0].set(bodyBarelyLoaded.getValue());
      isLinearAxisFeedbackControlled[1].set(bodyBarelyLoaded.getValue());
      isLinearAxisFeedbackControlled[2].set(false);

      isAngularAxisFeedbackControlled[0].set(false);
      isAngularAxisFeedbackControlled[1].set(false);
      isAngularAxisFeedbackControlled[2].set(true);

      // Selection matrices
      for (int axisIdx = 0; axisIdx < 3; axisIdx++)
      {
         spatialFeedbackSelectionMatrix.getLinearPart().selectAxis(axisIdx, loadBearingParameters.isLinearAxisEnabled(axisIdx) && isLinearAxisFeedbackControlled[axisIdx].getValue());
         spatialFeedbackSelectionMatrix.getAngularPart().selectAxis(axisIdx, loadBearingParameters.isAngularAxisEnabled(axisIdx) && isAngularAxisFeedbackControlled[axisIdx].getValue());

         spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(axisIdx, loadBearingParameters.isLinearAxisEnabled(axisIdx) && !isLinearAxisFeedbackControlled[axisIdx].getValue());
         spatialAccelerationSelectionMatrix.getAngularPart().selectAxis(axisIdx, loadBearingParameters.isAngularAxisEnabled(axisIdx) && !isAngularAxisFeedbackControlled[axisIdx].getValue());
      }

      spatialFeedbackControlCommand.setSelectionMatrix(spatialFeedbackSelectionMatrix);
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);

      double timeInTrajectory = getTimeInTrajectory();

      if (controlMode.getValue() == LoadBearingControlMode.JOINTSPACE)
      {
         jointControlHelper.doAction(timeInTrajectory);
      }
      else
      {
         orientationControlHelper.doAction(timeInTrajectory);
         spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(handLoadedLinearWeight);
      }

      updateGraphics();
   }

   public boolean handleLoadBearingCommand(LoadBearingCommand command)
   {
      coefficientOfFriction.set(command.getCoefficientOfFriction());

      // Initialize contact in body-frame
      contactPoseInBodyFrame.setIncludingFrame(bodyFrame, command.getContactPoseInBodyFrame());
      contactFrame.setPoseAndUpdate(contactPoseInBodyFrame);

      // Initialize contact in world-frame
      contactPoseInWorldFrame.setMatchingFrame(contactPoseInBodyFrame);
      contactPoseInWorldFrame.changeFrame(worldFrame);
      contactNormal.setMatchingFrame(contactFrame, Axis3D.Z);

      return true;
   }

   public boolean handleJointTrajectoryCommand(JointspaceTrajectoryCommand jointspaceCommand, double[] initialJointPositions)
   {
      if (jointControlHelper == null)
      {
         LogTools.warn(warningPrefix + "Cannot use jointspace mode. Was not created with a jointspace helper.");
         return false;
      }

      if (!handleCommandInternal(jointspaceCommand))
      {
         return false;
      }

      if (!jointControlHelper.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         return false;
      }

      controlMode.set(LoadBearingControlMode.JOINTSPACE);
      return true;
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand orientationCommand)
   {
      if (orientationControlHelper == null)
      {
         LogTools.warn(warningPrefix + "Cannot use orientation mode. Was not created with an orientation helper.");
         return false;
      }

      if (!handleCommandInternal(orientationCommand))
      {
         return false;
      }

      if (!orientationControlHelper.handleTrajectoryCommand(orientationCommand))
      {
         return false;
      }

      controlMode.set(LoadBearingControlMode.ORIENTATION);
      return true;
   }

   public PIDSE3Configuration getHoldPositionHandControlGains()
   {
      double kpXYZ = 150.0;
      double zetaXYZ = 0.7;

      double kpXYZOrientation = 50.0; // 150.0;
      double zetaOrientation = 0.8;
      double maxAcceleration = 8.0;
      double maxJerk = 100.0;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(kpXYZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxAcceleration, maxJerk);
      gains.setOrientationProportionalGains(kpXYZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAcceleration, maxJerk);

      return new PIDSE3Configuration(GainCoupling.XY, false, gains);
   }

   @Override
   public void onEntry()
   {

   }

   @Override
   public void onExit(double timeInState)
   {
      hideGraphics();
      controlMode.set(loadBearingParameters.getDefaultControlMode());

      desiredContactPointInWorld.setToNaN();
      desiredContactOrientationInWorld.setToNaN();
      currentContactPointInWorld.setToNaN();
      currentContactOrientationInWorld.setToNaN();

      // from RigidBodyJointspaceControlState.holdCurrent
      jointControlHelper.overrideTrajectory();
      jointControlHelper.setWeightsToDefaults();
      jointControlHelper.queueInitialPointsAtCurrent();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
      return feedbackControlCommandList;

      //      if (controlMode.getValue() == LoadBearingControlMode.JOINTSPACE)
      //      else
      //         feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
      return feedbackControlCommandList;

      //      feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
   }

   @Override
   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setHasContactStateChanged(true);
      return planeContactStateCommand;
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

   @Override
   public boolean isDone(double time)
   {
      double positionErrorSquared = EuclidCoreTools.normSquared(positionError.getX(), positionError.getY(), positionError.getZ());
      double linearTrackingSlipThresholdSquared = EuclidCoreTools.square(loadBearingParameters.getLinearTrackingSlipThreshold());
      return positionErrorSquared > linearTrackingSlipThresholdSquared;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(contactPoint.getNamePrefix(), currentContactPointInWorld, 0.01, ColorDefinitions.Black()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(contactNormal.getNamePrefix(),
                                                                    currentContactPointInWorld,
                                                                    contactNormal,
                                                                    0.1,
                                                                    ColorDefinitions.Black()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(bodyToControl.getName() + "ControllerForce",
                                                                    currentContactPointInWorld,
                                                                    yoControllerDesiredForce,
                                                                    0.01,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(bodyToControl.getName() + "ControlFrame",
                                                                               currentContactPointInWorld,
                                                                               currentContactOrientationInWorld,
                                                                               0.15,
                                                                               ColorDefinitions.DarkGray()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(bodyToControl.getName() + "DesiredControlFrame",
                                                                               desiredContactPointInWorld,
                                                                               desiredContactOrientationInWorld,
                                                                               0.13,
                                                                               ColorDefinitions.LightGray()));
      return group;
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public FramePoint3DReadOnly getContactPoint()
   {
      return contactPoint;
   }

   public FrameVector3DReadOnly getContactNormal()
   {
      return contactNormal;
   }

   public LoadBearingControlMode getLoadBearingControlMode()
   {
      return controlMode.getValue();
   }

   public void updateWholeBodyContactState(WholeBodyContactState wholeBodyContactStateToUpdate)
   {
      wholeBodyContactStateToUpdate.addContactPoints(planeContactStateCommand);
   }

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointControlHelper.getJointDesiredPosition(jointIdx);
   }
}
