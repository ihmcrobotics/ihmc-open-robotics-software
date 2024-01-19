package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.CommandConversionTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.*;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
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

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private final RigidBodyBasics bodyToControl;
   private final LoadBearingParameters loadBearingParameters;
   private final SelectionMatrix3D positionFeedbackSelectionMatrix = new SelectionMatrix3D();
   private final SelectionMatrix6D spatialAccelerationSelectionMatrix = new SelectionMatrix6D();

   /* Controller Commands */
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   /* Control gains, weights and axis selection */
   private final YoBoolean[] isLinearAxisFeedbackControlled = new YoBoolean[3];
   private final PID3DGainsReadOnly holdHandPositionGains;
   private final Vector3DReadOnly handLoadedLinearWeight = new Vector3D(5.0, 5.0, 5.0);

   /* Hand load status */
   private final YoBoolean bodyBarelyLoaded;
   private ControllerCoreOutputReadOnly controllerCoreOutput;
   private final Wrench controllerDesiredWrench = new Wrench();
   private final FrameVector3D controllerDesiredForce = new FrameVector3D();
   private final YoFrameVector3D yoControllerDesiredForce;

   /* Reference frames */
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final FramePoint3D contactPointInBody = new FramePoint3D();
   private final FramePose3D contactControlPoseInWorld = new FramePose3D();
   private final PoseReferenceFrame contactControlFrame;

   /* Yo-Contact frame components */
   private final YoFrameVector3D contactNormal;
   private final YoFramePoint3D currentContactPointInWorld;
   private final YoFramePoint3D yoContactControlFramePosition;
   private final YoFrameQuaternion yoContactControlFrameOrientation;
   private final YoFramePoint3D yoContactPointInBodyFrame;

   /* Trajectory handlers */
   private final RigidBodyJointControlHelper jointControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;
   private final SO3TrajectoryControllerCommand orientationTrajectoryCommand = new SO3TrajectoryControllerCommand();

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

      contactControlFrame = new PoseReferenceFrame("contactControlFrame" + bodyName, ReferenceFrame.getWorldFrame());
      bodyAcceleration = new SpatialAcceleration(contactControlFrame, elevatorFrame, contactControlFrame);
      pointFeedbackControlCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.set(elevator, bodyToControl);
      yoContactPointInBodyFrame = new YoFramePoint3D("contactPointInBody" + bodyName, bodyFrame, registry);

      // TODO move this up so left/right have same gains
      this.holdHandPositionGains = getHoldPositionHandControlGains();

      coefficientOfFriction = new YoDouble(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector3D(bodyName + "ContactNormal", worldFrame, parentRegistry);
      currentContactPointInWorld = new YoFramePoint3D(bodyName + "currentContactPoint", worldFrame, registry);
      yoContactControlFramePosition = new YoFramePoint3D(bodyName + "contactControlFramePosition", worldFrame, registry);
      yoContactControlFrameOrientation = new YoFrameQuaternion(bodyName + "contactControlFrameOrientation", worldFrame, registry);

      bodyBarelyLoaded = new YoBoolean(bodyName + "BarelyLoaded", registry);
      yoControllerDesiredForce = new YoFrameVector3D(bodyName + "DesiredForce", ReferenceFrame.getWorldFrame(), parentRegistry);

      positionError = new YoFramePoint3D(bodyName + "PositionError", contactControlFrame, registry);
      orientationError = new YoFrameQuaternion(bodyName + "OrientationError", contactControlFrame, registry);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      this.loadBearingParameters = new LoadBearingParameters(bodyName, registry);
      this.jointControlHelper = jointControlHelper;
      this.orientationControlHelper = orientationControlHelper;

      for (int i = 0; i < 3; i++)
      {
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

      YoGraphicCoordinateSystem contactControlFrame = new YoGraphicCoordinateSystem(bodyName + "LoadBearingControlFrame",
                                                                                    yoContactControlFramePosition,
                                                                                    yoContactControlFrameOrientation,
                                                                                    0.17,
                                                                                    YoAppearance.LightGray());
      graphicsListRegistry.registerYoGraphic(listName, contactControlFrame);
      graphics.add(contactControlFrame);

      hideGraphics();
   }

   @Override
   public void doAction(double timeInState)
   {
      currentContactPointInWorld.setMatchingFrame(contactPointInBody);

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
      planeContactStateCommand.addPointInContact(getYoContactPointInBodyFrame());
      planeContactStateCommand.setHasContactStateChanged(false);

      positionError.setMatchingFrame(yoContactControlFramePosition);
      orientationError.setMatchingFrame(yoContactControlFrameOrientation);

      // assemble spatial feedback command
      pointFeedbackControlCommand.setBodyFixedPointToControl(contactPointInBody);
      pointFeedbackControlCommand.setGainsFrame(contactControlFrame);

      tempVector.setToZero(ReferenceFrame.getWorldFrame());
      pointFeedbackControlCommand.setInverseDynamics(contactControlPoseInWorld.getPosition(), tempVector, tempVector);
      pointFeedbackControlCommand.setWeightsForSolver(handLoadedLinearWeight);
      pointFeedbackControlCommand.setGains(holdHandPositionGains);

      // assemble zero acceleration command
      bodyAcceleration.setToZero(contactControlFrame, elevatorFrame, contactControlFrame);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(contactControlFrame, bodyAcceleration);

      // set axis control strategies
      isLinearAxisFeedbackControlled[0].set(bodyBarelyLoaded.getValue());
      isLinearAxisFeedbackControlled[1].set(bodyBarelyLoaded.getValue());
      isLinearAxisFeedbackControlled[2].set(false);

      // Selection matrices
      for (int axisIdx = 0; axisIdx < 3; axisIdx++)
      {
         positionFeedbackSelectionMatrix.selectAxis(axisIdx, loadBearingParameters.isLinearAxisEnabled(axisIdx) && isLinearAxisFeedbackControlled[axisIdx].getValue());
         spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(axisIdx, loadBearingParameters.isLinearAxisEnabled(axisIdx) && !isLinearAxisFeedbackControlled[axisIdx].getValue());
         spatialAccelerationSelectionMatrix.getAngularPart().selectAxis(axisIdx, false);
      }

      pointFeedbackControlCommand.setSelectionMatrix(positionFeedbackSelectionMatrix);
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);

      double timeInTrajectory = getTimeInTrajectory();

      // TODO
      jointControlHelper.doAction(timeInTrajectory);
//      orientationControlHelper.doAction(timeInTrajectory);
      spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(handLoadedLinearWeight);

      updateGraphics();
   }

   public void load(double coefficientOfFriction, Point3D contactPointInBodyFrame, Vector3D contactNormalInWorldFrame)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);

      // Initialize contact point and contact normal
      this.contactPointInBody.setIncludingFrame(bodyFrame, contactPointInBodyFrame);
      this.contactNormal.set(contactNormalInWorldFrame);

      // Compute contact control frame, which  is static in world, has an origin at the contact point and has Z pointing parallel to the contact normal
      contactControlPoseInWorld.setReferenceFrame(ReferenceFrame.getWorldFrame());
      contactControlPoseInWorld.getPosition().setMatchingFrame(this.contactPointInBody);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, contactNormalInWorldFrame, contactControlPoseInWorld.getOrientation());
      contactControlFrame.setPoseAndUpdate(contactControlPoseInWorld);

      // Update yovariables
      yoContactControlFramePosition.setMatchingFrame(contactControlPoseInWorld.getPosition());
      yoContactControlFrameOrientation.setMatchingFrame(contactControlPoseInWorld.getOrientation());
      this.yoContactPointInBodyFrame.set(contactPointInBodyFrame);
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

      return true;
   }

   public boolean handleOrientationTrajectoryCommand(SE3TrajectoryControllerCommand taskspaceTrajectory)
   {
      if (orientationControlHelper == null)
      {
         LogTools.warn(warningPrefix + "Cannot use orientation mode. Was not created with an orientation helper.");
         return false;
      }

      if (!handleCommandInternal(taskspaceTrajectory))
      {
         return false;
      }

      CommandConversionTools.convertToSO3(taskspaceTrajectory, orientationTrajectoryCommand);
      if (!orientationControlHelper.handleTrajectoryCommand(orientationTrajectoryCommand))
      {
         return false;
      }

      return true;
   }

   public ParameterizedPID3DGains getHoldPositionHandControlGains()
   {
      GainCoupling gainCoupling = GainCoupling.XY;
      double kpXYZ = 150.0;
      double zetaXYZ = 0.7;
      double maxAcceleration = 8.0;
      double maxJerk = 100.0;

      DefaultPID3DGains holdPositionGains = new DefaultPID3DGains();
      holdPositionGains.setProportionalGains(kpXYZ);
      holdPositionGains.setDampingRatios(zetaXYZ);
      holdPositionGains.setMaxFeedbackAndFeedbackRate(maxAcceleration, maxJerk);

      return new ParameterizedPID3DGains("Hold" + bodyToControl.getName(), new PID3DConfiguration(gainCoupling, false, holdPositionGains), registry);
   }

   @Override
   public void onEntry()
   {

   }

   @Override
   public void onExit(double timeInState)
   {
      hideGraphics();

      yoContactControlFramePosition.setToNaN();
      yoContactControlFrameOrientation.setToNaN();
      currentContactPointInWorld.setToNaN();

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
      feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
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
      feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
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
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(yoContactPointInBodyFrame.getNamePrefix(),
                                                                    currentContactPointInWorld,
                                                                    0.01,
                                                                    ColorDefinitions.Black()));
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
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(bodyToControl.getName() + "ContactControlFrame",
                                                                               yoContactControlFramePosition,
                                                                               yoContactControlFrameOrientation,
                                                                               0.13,
                                                                               ColorDefinitions.LightGray()));
      return group;
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public FramePoint3DReadOnly getYoContactPointInBodyFrame()
   {
      return yoContactPointInBodyFrame;
   }

   public FrameVector3DReadOnly getContactNormal()
   {
      return contactNormal;
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
