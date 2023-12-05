package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
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

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   private ControllerCoreOutputReadOnly controllerCoreOutput;
   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();
   private final YoBoolean[] isAngularAxisFeedbackControlled = new YoBoolean[3];
   private final YoBoolean[] isLinearAxisFeedbackControlled = new YoBoolean[3];

   private final FramePose3D bodyFixedControlledPose = new FramePose3D();
   private final SpatialAcceleration bodyAcceleration;

   private final FramePoint3D previousContactPoint = new FramePoint3D();
   private final FrameVector3D previousContactNormal = new FrameVector3D();
   private double previousCoefficientOfFriction;

   private final YoFramePoint3D contactPoint;
   private final YoFramePoint3D contactPointInWorld;
   private final YoFrameQuaternion controlFrameOrientationInWorld;

   private final YoFramePoint3D yoDesiredContactPointInWorld;
   private final YoFrameQuaternion yoDesiredControlFrameOrientationInWorld;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;

   private final ContactablePlaneBody contactableBody;
   private final YoDouble coefficientOfFriction;
   private final YoFrameVector3D contactNormal;
   private final ReferenceFrame contactFrame;
   private final YoEnum<LoadBearingControlMode> controlMode;

   private final PIDSE3GainsReadOnly holdHandPositionGains;
   private final Vector3DReadOnly handLoadedLinearWeight;
   private final Vector3DReadOnly handLoadedAngularWeight;

   private final RigidBodyTransform bodyToJointTransform = new RigidBodyTransform();
   private final RigidBodyTransform contactToJointTransform = new RigidBodyTransform();

   private final FramePoint3D desiredContactPosition = new FramePoint3D(worldFrame);
   private final FrameQuaternion desiredContactOrientation = new FrameQuaternion(worldFrame);
   private final FramePoint3D currentContactPosition = new FramePoint3D(worldFrame);
   private final FrameQuaternion currentContactOrientation = new FrameQuaternion(worldFrame);

   private final RigidBodyJointControlHelper jointControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;

   private final LoadBearingParameters loadBearingParameters;
   private final YoBoolean bodyBarelyLoaded;
   private final Wrench controllerDesiredWrench = new Wrench();
   private final FrameVector3D controllerDesiredForce = new FrameVector3D();
   private final YoFrameVector3D yoControllerDesiredForce;
   private final FramePose3D controlFrameInWorld = new FramePose3D();

   public RigidBodyLoadBearingControlState(RigidBodyBasics bodyToControl,
                                           ContactablePlaneBody contactableBody,
                                           RigidBodyBasics elevator,
                                           YoDouble yoTime,
                                           RigidBodyJointControlHelper jointControlHelper,
                                           RigidBodyOrientationControlHelper orientationControlHelper,
                                           PIDSE3GainsReadOnly holdHandPositionGains,
                                           Vector3DReadOnly handLoadedLinearWeight,
                                           Vector3DReadOnly handLoadedAngularWeight,
                                           YoGraphicsListRegistry graphicsListRegistry,
                                           YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.contactFrame = contactableBody.getSoleFrame();
      this.contactableBody = contactableBody;

      bodyFrame.getTransformToDesiredFrame(bodyToJointTransform, bodyToControl.getParentJoint().getFrameAfterJoint());
      bodyAcceleration = new SpatialAcceleration(contactFrame, elevatorFrame, contactFrame);
      spatialFeedbackControlCommand.set(elevator, bodyToControl);

      spatialAccelerationCommand.set(elevator, bodyToControl);
      this.handLoadedLinearWeight = handLoadedLinearWeight;
      this.handLoadedAngularWeight = handLoadedAngularWeight;
      // TODO move this up so left/right have same gains
      this.holdHandPositionGains = new ParameterizedPIDSE3Gains("Hold" + bodyToControl.getName(), getHoldPositionHandControlGains(), parentRegistry);

      String bodyName = bodyToControl.getName();
      coefficientOfFriction = new YoDouble(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector3D(bodyName + "ContactNormal", worldFrame, parentRegistry);
      contactPoint = new YoFramePoint3D(bodyName + "ContactPoint", contactFrame, parentRegistry);
      contactPointInWorld = new YoFramePoint3D(bodyName + "ContactPointInWorld", worldFrame, parentRegistry);
      controlFrameOrientationInWorld = new YoFrameQuaternion(bodyName + "ContactFrameRotationInWorld", worldFrame, parentRegistry);
      yoDesiredContactPointInWorld = new YoFramePoint3D(bodyName + "DesiredContactPointInWorld", worldFrame, parentRegistry);
      yoDesiredControlFrameOrientationInWorld = new YoFrameQuaternion(bodyName + "DesiredContactFrameRotationInWorld", worldFrame, parentRegistry);

      controlMode = new YoEnum<>(bodyName + "ControlMode", parentRegistry, LoadBearingControlMode.class);
      bodyBarelyLoaded = new YoBoolean(bodyName + "BarelyLoaded", registry);
      yoControllerDesiredForce = new YoFrameVector3D(bodyName + "DesiredForce", ReferenceFrame.getWorldFrame(), parentRegistry);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      this.loadBearingParameters = new LoadBearingParameters(bodyName, parentRegistry);
      this.jointControlHelper = jointControlHelper;
      this.orientationControlHelper = orientationControlHelper;

      for (int i = 0; i < 3; i++)
      {
         isAngularAxisFeedbackControlled[i] = new YoBoolean("isAngular" + Axis3D.values[i] + "FeedbackControlled", parentRegistry);
         isLinearAxisFeedbackControlled[i] = new YoBoolean("isLinear" + Axis3D.values[i] + "FeedbackControlled", parentRegistry);
      }

//      setupViz(graphicsListRegistry, bodyName);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry, String bodyName)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();

      YoGraphicVector surfaceNormal = new YoGraphicVector(bodyName + "ContactNormal", contactPointInWorld, contactNormal, 0.1, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, surfaceNormal);
      graphics.add(surfaceNormal);

      YoGraphicPosition contactPoint = new YoGraphicPosition(bodyName + "ContactPoint", contactPointInWorld, 0.01, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, contactPoint);
      graphics.add(contactPoint);

      YoGraphicVector controllerDesiredForce = new YoGraphicVector(bodyName + "DesiredForceGraphic", contactPointInWorld, yoControllerDesiredForce, 0.015, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic(listName, controllerDesiredForce);
      graphics.add(controllerDesiredForce);

      YoGraphicCoordinateSystem controlFrame = new YoGraphicCoordinateSystem(bodyName + "LoadBearingControlFrame", contactPointInWorld, controlFrameOrientationInWorld, 0.2);
      graphicsListRegistry.registerYoGraphic(listName, controlFrame);
      graphics.add(controlFrame);

      hideGraphics();
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalInWorldFrame(Vector3D contactNormalInWorldFrame)
   {
      contactNormal.set(contactNormalInWorldFrame);
   }

   public void setAndUpdateContactFrame(RigidBodyTransform bodyToContactFrame)
   {
      contactToJointTransform.set(bodyToJointTransform);
      contactToJointTransform.multiply(bodyToContactFrame);
      contactableBody.setSoleFrameTransformFromParentJoint(contactToJointTransform); // This updates contactFrame
      contactPoint.setToZero();
   }

   @Override
   public void doAction(double timeInState)
   {
      contactFrame.update();

      controlFrameInWorld.setToZero(contactFrame);
      controlFrameInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      contactPointInWorld.set(controlFrameInWorld.getPosition());
      controlFrameOrientationInWorld.set(controlFrameInWorld.getOrientation());

      if (controllerCoreOutput.getDesiredExternalWrench(controllerDesiredWrench, contactableBody.getRigidBody()))
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

      // set axis control strategies
      isLinearAxisFeedbackControlled[0].set(bodyBarelyLoaded.getValue());
      isLinearAxisFeedbackControlled[1].set(bodyBarelyLoaded.getValue());
      isLinearAxisFeedbackControlled[2].set(false);

      isAngularAxisFeedbackControlled[0].set(true);
      isAngularAxisFeedbackControlled[1].set(true);
      isAngularAxisFeedbackControlled[2].set(true);

      // assemble contact command
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal);
      planeContactStateCommand.addPointInContact(contactPoint);
      planeContactStateCommand.setHasContactStateChanged(hasContactStateNotChanged());

      if (bodyBarelyLoaded.getValue())
      { // assemble spatial feedback command
         currentContactPosition.setToZero(contactFrame);
         currentContactOrientation.setToZero(contactFrame);
         currentContactPosition.changeFrame(desiredContactPosition.getReferenceFrame());
         currentContactOrientation.changeFrame(desiredContactOrientation.getReferenceFrame());

         bodyFixedControlledPose.setToZero(contactFrame);
         bodyFixedControlledPose.changeFrame(bodyFrame);
         spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
         spatialFeedbackControlCommand.setInverseDynamics(desiredContactOrientation, desiredContactPosition, zeroInWorld, zeroInWorld, zeroInWorld, zeroInWorld);

         spatialFeedbackControlCommand.setWeightsForSolver(handLoadedAngularWeight, handLoadedLinearWeight);
         spatialFeedbackControlCommand.setOrientationGains(holdHandPositionGains.getOrientationGains());
         spatialFeedbackControlCommand.setPositionGains(holdHandPositionGains.getPositionGains());

//         spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);
//
//         spatialFeedbackControlCommand.setOrientationGains(taskspaceOrientationGains);
//         spatialFeedbackControlCommand.setPositionGains(taskspacePositionGains);
//         spatialFeedbackControlCommand.setWeightsForSolver(taskspaceAngularWeight, taskspaceLinearWeight);


      }
      else
      { // assemble zero acceleration command
         bodyAcceleration.setToZero(contactFrame, elevatorFrame, contactFrame);
         bodyAcceleration.setBodyFrame(bodyFrame);
         spatialAccelerationCommand.setSpatialAcceleration(contactFrame, bodyAcceleration);
//         accelerationSelectionMatrix.resetSelection();
//         spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);
         spatialAccelerationCommand.setSelectionMatrixForLinearControl();

         isLinearAxisFeedbackControlled[0].set(false);
         isLinearAxisFeedbackControlled[1].set(false);
         isLinearAxisFeedbackControlled[2].set(false);

         isAngularAxisFeedbackControlled[0].set(false);
         isAngularAxisFeedbackControlled[1].set(false);
         isAngularAxisFeedbackControlled[2].set(true);
      }

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

      previousContactNormal.setIncludingFrame(contactNormal);
      previousContactPoint.setIncludingFrame(contactPoint);
      previousCoefficientOfFriction = coefficientOfFriction.getDoubleValue();

//      updateGraphics();
   }

   public boolean handleLoadbearingCommand(LoadBearingCommand command)
   {
      setCoefficientOfFriction(command.getCoefficientOfFriction());
      setContactNormalInWorldFrame(command.getContactNormalInWorldFrame());
      setAndUpdateContactFrame(command.getBodyFrameToContactFrame());
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
      double kpXY = 100.0;
      double kpZ = 100.0;
      double zetaXYZ = 0.4;
      double kpXYOrientation = 100.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 0.4;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return new PIDSE3Configuration(GainCoupling.XY, false, gains);
   }


   @Override
   public void onEntry()
   {
      desiredContactPosition.setToZero(contactFrame);
      desiredContactOrientation.setToZero(contactFrame);

      desiredContactPosition.changeFrame(worldFrame);
      desiredContactOrientation.changeFrame(worldFrame);

      yoDesiredContactPointInWorld.set(desiredContactPosition);
      yoDesiredControlFrameOrientationInWorld.set(desiredContactOrientation);

      previousContactPoint.setToZero(worldFrame);
      previousContactNormal.setToZero(worldFrame);
      previousCoefficientOfFriction = 0.0;
   }

   private boolean hasContactStateNotChanged()
   {
      boolean hasContactStateNotChanged = previousContactNormal.equals(contactNormal);
      hasContactStateNotChanged &= previousContactPoint.equals(contactPoint);
      hasContactStateNotChanged &= previousCoefficientOfFriction == coefficientOfFriction.getDoubleValue();

      return hasContactStateNotChanged;
   }

   @Override
   public void onExit(double timeInState)
   {
      hideGraphics();
      controlMode.set(loadBearingParameters.getDefaultControlMode());
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      if (!bodyBarelyLoaded.getValue())
         inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      if (controlMode.getValue() == LoadBearingControlMode.JOINTSPACE)
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      else
         feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
      if (bodyBarelyLoaded.getValue())
         feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
      return feedbackControlCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
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
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(contactPoint.getNamePrefix(), contactPointInWorld, 0.01, ColorDefinitions.Black()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(contactNormal.getNamePrefix(),
                                                                    contactPointInWorld,
                                                                    contactNormal, 0.1, ColorDefinitions.Black()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(contactableBody.getRigidBody().getName() + "ControllerForce",
                                                                    contactPointInWorld,
                                                                    yoControllerDesiredForce,
                                                                    0.01,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(contactableBody.getRigidBody().getName() + "ControlFrame",
                                                                               contactPointInWorld,
                                                                               controlFrameOrientationInWorld,
                                                                               0.15,
                                                                               ColorDefinitions.DarkGray()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(contactableBody.getRigidBody().getName() + "DesiredControlFrame",
                                                                               yoDesiredContactPointInWorld,
                                                                               yoDesiredControlFrameOrientationInWorld,
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
