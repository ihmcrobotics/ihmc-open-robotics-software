package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
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

/**
 * A load bearing state for a rigid body. This is currently for using the hands
 * to help support the weight of the robot. It is based on a contact model
 * of a single point on the hand contacting an environmental plane.
 */
public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{
   // Debug variables for toggling various objectives
   private static final boolean ENABLE_CONTACT = true;
   private static final boolean ENABLE_ZERO_ACCELERATION = true;
   private static final boolean ENABLE_POINT_FEEDBACK = true;
   private static final boolean ENABLE_JOINTSPACE_FEEDBACK = true;
   private static final boolean ENABLE_ORIENTATION_FEEDBACK = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3D zeroWorld = new FrameVector3D();

   /* Controller Commands */
   private final RigidBodyBasics bodyToControl;
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   /* Control gains, weights and axis selection */
   private final LoadBearingParameters loadBearingParameters;
   private final Vector3DReadOnly linearWeight = new Vector3D(50.0, 50.0, 50.0);
   private final Vector3DReadOnly angularWeight = new Vector3D(5.0, 5.0, 5.0);
   private final DefaultYoPIDSE3Gains feedbackGains;
   private final SelectionMatrix3D positionFeedbackSelectionMatrix = new SelectionMatrix3D();
   private final SelectionMatrix6D spatialAccelerationSelectionMatrix = new SelectionMatrix6D();

   /* Hand load status */
   private final GlitchFilteredYoBoolean bodyBarelyLoaded;
   private ControllerCoreOutputReadOnly controllerCoreOutput;
   private final Wrench controllerDesiredWrench = new Wrench();
   private final FrameVector3D controllerDesiredForce = new FrameVector3D();
   private final YoFrameVector3D yoControllerDesiredForce;

   /* Active control modes */
   private final YoBoolean jointspaceControlActive;
   private final YoBoolean orientationControlActive;

   /* Reference frames */
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final FramePoint3D contactPointInBody = new FramePoint3D();
   private final FramePose3D desiredContactPoseWorld = new FramePose3D();
   private final PoseReferenceFrame desiredContactFrameFixedInWorld;

   /* Yo-Contact frame components */
   private final YoFrameVector3D contactNormal;
   private final YoFramePoint3D currentContactPointInWorld;
   private final YoFramePoint3D yoDesiredContactPosition;
   private final YoFrameQuaternion yoDesiredContactOrientation;
   private final YoFramePoint3D yoContactPointInBodyFrame;

   /* Trajectory handlers */
   private final RigidBodyJointControlHelper jointControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;
   private final SO3TrajectoryControllerCommand orientationTrajectoryCommand = new SO3TrajectoryControllerCommand();

   private final YoDouble coefficientOfFriction;
   private final SpatialAcceleration bodyAcceleration;

   /* Error measurement to detect slipping */
   private final YoFramePoint3D positionError;

   public RigidBodyLoadBearingControlState(RigidBodyBasics bodyToControl,
                                           RigidBodyBasics baseBody,
                                           RigidBodyBasics elevator,
                                           YoDouble yoTime,
                                           RigidBodyJointControlHelper jointControlHelper,
                                           RigidBodyOrientationControlHelper orientationControlHelper,
                                           LoadBearingParameters loadBearingParameters,
                                           YoGraphicsListRegistry graphicsListRegistry,
                                           YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);

      this.bodyToControl = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.loadBearingParameters = loadBearingParameters;

      LogTools.info("Setting up load bearing state " + bodyToControl.getName());

      String bodyName = bodyToControl.getName();

      desiredContactFrameFixedInWorld = new PoseReferenceFrame("desiredContactFrame" + bodyName, ReferenceFrame.getWorldFrame());
      bodyAcceleration = new SpatialAcceleration(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);

      pointFeedbackControlCommand.set(elevator, bodyToControl);
      pointFeedbackControlCommand.setPrimaryBase(baseBody);

      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setPrimaryBase(baseBody);

      feedbackGains = new DefaultYoPIDSE3Gains("LoadBearing", GainCoupling.XY, false, parentRegistry);
      configureGains();

      yoContactPointInBodyFrame = new YoFramePoint3D("contactPointInBody" + bodyName, bodyFrame, registry);

      coefficientOfFriction = new YoDouble(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector3D(bodyName + "ContactNormal", worldFrame, parentRegistry);
      currentContactPointInWorld = new YoFramePoint3D(bodyName + "currentContactPoint", worldFrame, registry);
      yoDesiredContactPosition = new YoFramePoint3D(bodyName + "_DesiredContactPosition", worldFrame, registry);
      yoDesiredContactOrientation = new YoFrameQuaternion(bodyName + "_DesiredContactOrientation", worldFrame, registry);

      bodyBarelyLoaded = new GlitchFilteredYoBoolean(bodyName + "BarelyLoaded", registry, 60);
      jointspaceControlActive = new YoBoolean(bodyName + "JointspaceControlActive", registry);
      orientationControlActive = new YoBoolean(bodyName + "OrientationControlActive", registry);
      yoControllerDesiredForce = new YoFrameVector3D(bodyName + "DesiredForce", ReferenceFrame.getWorldFrame(), parentRegistry);

      positionError = new YoFramePoint3D(bodyName + "PositionError", desiredContactFrameFixedInWorld, registry);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      this.jointControlHelper = jointControlHelper;
      this.orientationControlHelper = orientationControlHelper;

      setupViz(graphicsListRegistry, bodyName);
   }

   private void configureGains()
   {
      double kpXYPosition = 100.0;
      double kpZPosition = 0.0;
      double zetaXYPosition = 1.0;
      double kdXYPosition = GainCalculator.computeDerivativeGain(kpXYPosition, zetaXYPosition);
      double kdZ = 0.0;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      feedbackGains.setPositionProportionalGains(kpXYPosition, kpXYPosition, kpZPosition);
      feedbackGains.setPositionDerivativeGains(kdXYPosition, kdXYPosition, kdZ);
      feedbackGains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);

      double kpXYOrientation = 100.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 1.0;
      double kdOrientationXY = GainCalculator.computeDerivativeGain(kpXYOrientation, zetaOrientation);
      double kdOrientationZ = GainCalculator.computeDerivativeGain(kpZOrientation, zetaOrientation);
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;
      feedbackGains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      feedbackGains.setOrientationDerivativeGains(kdOrientationXY, kdOrientationXY, kdOrientationZ);
      feedbackGains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);
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

      YoGraphicCoordinateSystem contactControlFrame = new YoGraphicCoordinateSystem(bodyName + "LoadBearingControlFrame", yoDesiredContactPosition,
                                                                                    yoDesiredContactOrientation,
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
         bodyBarelyLoaded.update(desiredForceLoadMagnitudeSquared < MathTools.square(loadBearingParameters.getNormalForceThresholdForLoaded()));

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

      // assemble zero acceleration command
      bodyAcceleration.setToZero(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(desiredContactFrameFixedInWorld, bodyAcceleration);
      spatialAccelerationSelectionMatrix.getAngularPart().clearSelection();
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.X.ordinal(), !bodyBarelyLoaded.getValue());
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Y.ordinal(), !bodyBarelyLoaded.getValue());
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Z.ordinal(), true);
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);
      spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(linearWeight);

      // record contact point tracking error
      positionError.setMatchingFrame(contactPointInBody);

      // assemble spatial feedback command
      if (bodyBarelyLoaded.getValue())
      {
         pointFeedbackControlCommand.setBodyFixedPointToControl(contactPointInBody);
         pointFeedbackControlCommand.setGainsFrame(desiredContactFrameFixedInWorld);
         pointFeedbackControlCommand.setInverseDynamics(desiredContactPoseWorld.getPosition(), zeroWorld, zeroWorld);
         pointFeedbackControlCommand.setWeightsForSolver(linearWeight);

         double kp = loadBearingParameters.getHoldPositionStiffness();
         double zeta = loadBearingParameters.getHoldPositionDampingRatio();
         double kd = GainCalculator.computeDerivativeGain(kp, zeta);
         pointFeedbackControlCommand.getGains().setProportialAndDerivativeGains(kp, kd);

         positionFeedbackSelectionMatrix.selectAxis(Axis3D.X.ordinal(), true);
         positionFeedbackSelectionMatrix.selectAxis(Axis3D.Y.ordinal(), true);
         positionFeedbackSelectionMatrix.selectAxis(Axis3D.Z.ordinal(), false);
         pointFeedbackControlCommand.setSelectionMatrix(positionFeedbackSelectionMatrix);
      }

      // update jointspace and orientation trajectories if active
      double timeInTrajectory = getTimeInTrajectory();
      if (jointspaceControlActive.getValue())
      {
         jointControlHelper.doAction(timeInTrajectory);
      }
      if (orientationControlActive.getValue())
      {
         orientationControlHelper.doAction(timeInTrajectory);
      }

      updateGraphics();
   }

   public void load(double coefficientOfFriction,
                    Point3D contactPointInBodyFrame,
                    Vector3D contactNormalInWorldFrame,
                    boolean jointspaceControlActive,
                    boolean orientationControlActive)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);

      // Initialize contact point and contact normal
      this.contactPointInBody.setIncludingFrame(bodyFrame, contactPointInBodyFrame);
      this.contactNormal.set(contactNormalInWorldFrame);

      // Compute desired contact pose, which is static in world, has an origin at the contact point and has Z pointing parallel to the contact normal
      desiredContactPoseWorld.setReferenceFrame(ReferenceFrame.getWorldFrame());
      desiredContactPoseWorld.getPosition().setMatchingFrame(this.contactPointInBody);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, contactNormalInWorldFrame, desiredContactPoseWorld.getOrientation());
      desiredContactFrameFixedInWorld.setPoseAndUpdate(desiredContactPoseWorld);

      // Update yovariables
      yoDesiredContactPosition.setMatchingFrame(desiredContactPoseWorld.getPosition());
      yoDesiredContactOrientation.setMatchingFrame(desiredContactPoseWorld.getOrientation());
      this.yoContactPointInBodyFrame.set(contactPointInBodyFrame);

      // Update active control modes
      this.jointspaceControlActive.set(jointspaceControlActive);
      this.orientationControlActive.set(orientationControlActive);
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

   public boolean handleAsOrientationTrajectoryCommand(SE3TrajectoryControllerCommand taskspaceTrajectory)
   { // Since HandTrajectoryMessage has an SE3 field this is a helper method to process that command.
      CommandConversionTools.convertToSO3(taskspaceTrajectory, orientationTrajectoryCommand);
      return handleOrientationTrajectoryCommand(orientationTrajectoryCommand);
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand orientationTrajectory)
   {
      if (orientationControlHelper == null)
      {
         LogTools.warn(warningPrefix + "Cannot use orientation mode. Was not created with an orientation helper.");
         return false;
      }

      if (!handleCommandInternal(orientationTrajectory))
      {
         return false;
      }

      if (!orientationControlHelper.handleTrajectoryCommand(orientationTrajectoryCommand))
      {
         return false;
      }

      return true;
   }

   @Override
   public void onEntry()
   {
      // Set to barely loaded during first tick
      bodyBarelyLoaded.set(true);

      // Reset trajectory time
      trajectoryDone.set(false);
      setTrajectoryStartTimeToCurrentTime();

      // Reset joint trajectory
      if (jointspaceControlActive.getValue())
      {
         jointControlHelper.overrideTrajectory();
         jointControlHelper.startTrajectoryExecution();
         jointControlHelper.queueInitialPointsAtCurrentDesired();
      }

      // Reset orientation trajectory
      if (orientationControlActive.getValue())
      {
         orientationControlHelper.holdCurrentDesired();
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      hideGraphics();

      yoDesiredContactPosition.setToNaN();
      yoDesiredContactOrientation.setToNaN();
      currentContactPointInWorld.setToNaN();

      jointControlHelper.overrideTrajectory();
      jointControlHelper.setWeightsToDefaults();
      jointControlHelper.startTrajectoryExecution();

      if (jointspaceControlActive.getValue())
      {
         jointControlHelper.queueInitialPointsAtCurrentDesired();
      }
      else
      {
         jointControlHelper.queueInitialPointsAtCurrent();
      }

      orientationControlHelper.clear();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();

      if (ENABLE_CONTACT)
      {
         inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      }
      if (ENABLE_ZERO_ACCELERATION)
      {
         inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      }

      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();

      if (ENABLE_POINT_FEEDBACK && bodyBarelyLoaded.getValue())
      {
         pointFeedbackControlCommand.setGains(feedbackGains.getPositionGains());
         feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
      }
      if (ENABLE_JOINTSPACE_FEEDBACK && jointspaceControlActive.getValue())
      {
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      }
      if (ENABLE_ORIENTATION_FEEDBACK && orientationControlActive.getValue())
      {
         OrientationFeedbackControlCommand orientationFeedbackCommand = orientationControlHelper.getFeedbackControlCommand();
         orientationFeedbackCommand.setWeightsForSolver(angularWeight);
         orientationFeedbackCommand.setGains(feedbackGains.getOrientationGains());
         feedbackControlCommandList.addCommand(orientationFeedbackCommand);
      }

      return feedbackControlCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();

      if (ENABLE_POINT_FEEDBACK)
      {
         feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
      }
      if (ENABLE_JOINTSPACE_FEEDBACK)
      {
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      }
      if (ENABLE_ORIENTATION_FEEDBACK)
      {
         feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
      }

      return feedbackControlCommandList;
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
                                                                    0.025,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(contactNormal.getNamePrefix(),
                                                                    currentContactPointInWorld,
                                                                    contactNormal,
                                                                    0.1,
                                                                    ColorDefinitions.Black()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(bodyToControl.getName() + "ControllerForce",
                                                                    currentContactPointInWorld,
                                                                    yoControllerDesiredForce,
                                                                    0.0075,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(bodyToControl.getName() + "ContactControlFrame", yoDesiredContactPosition,
                                                                               yoDesiredContactOrientation,
                                                                               0.12,
                                                                               ColorDefinitions.LightGray()));
      return group;
   }

   public boolean isJointspaceControlActive()
   {
      return jointspaceControlActive.getValue();
   }

   public boolean isOrientationControlActive()
   {
      return orientationControlActive.getValue();
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
      if (ENABLE_CONTACT)
      {
         wholeBodyContactStateToUpdate.addContactPoints(planeContactStateCommand);
      }
   }

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointControlHelper.getJointDesiredPosition(jointIdx);
   }
}
