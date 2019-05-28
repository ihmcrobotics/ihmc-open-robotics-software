package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is the active foot state when the foot is in flat support. Usually the command to the QP
 * should be a zero acceleration command. When the foot is barely loaded or the CoP gets close
 * to the edges of the foot polygon some of the directions start becoming feedback controlled. E.g.
 * when barely loaded x and y position as well as foot yaw are controlled to remain constant.
 *
 * The state also contains the ability to shift the CoP around within the foothold in case the
 * support area needs to be explored.
 */
public class SupportState extends AbstractFootControlState
{
   private static final double defaultFootLoadThreshold = 0.2;
   private static final int dofs = Twist.SIZE;

   private final YoVariableRegistry registry;

   private final FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D();

   private final YoBoolean footBarelyLoaded;
   private final YoBoolean copOnEdge;
   private final YoDouble footLoadThreshold;
   private final boolean[] isDirectionFeedbackControlled = new boolean[dofs];

   private final FootSwitchInterface footSwitch;

   private final PoseReferenceFrame controlFrame;
   private final PoseReferenceFrame desiredSoleFrame;
   private final YoGraphicReferenceFrame frameViz;

   private final InverseDynamicsCommandList inverseDynamicsCommandsList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackSelectionMatrix = new SelectionMatrix6D();

   private final FramePoint2D cop2d = new FramePoint2D();
   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameQuaternion frameOrientation = new FrameQuaternion();
   private final FramePose3D bodyFixedControlledPose = new FramePose3D();
   private final FramePoint3D desiredCopPosition = new FramePoint3D();

   private final FramePoint2D cop = new FramePoint2D();
   private final FramePoint2D desiredCoP = new FramePoint2D();

   private final FramePoint3D footPosition = new FramePoint3D();
   private final FrameQuaternion footOrientation = new FrameQuaternion();

   // For testing:
   private final BooleanProvider assumeCopOnEdge;
   private final BooleanProvider assumeFootBarelyLoaded;
   private final BooleanProvider neverHoldRotation;
   private final BooleanProvider neverHoldPosition;

   // For line contact walking and balancing:
   private final BooleanProvider holdFootOrientationFlat;

   // For foothold exploration:
   private final ExplorationHelper explorationHelper;
   private final PartialFootholdControlModule partialFootholdControlModule;
   private final YoBoolean requestFootholdExploration;
   private final YoDouble recoverTime;
   private final YoDouble timeBeforeExploring;

   // For straight legs with privileged configuration
   private final RigidBodyBasics pelvis;

   // Toe contact point loading time
   private final boolean rampUpAllowableToeLoadAfterContact;
   private final YoDouble toeLoadingDuration;
   private final YoDouble fullyLoadedMagnitude;

   private final FramePoint3D tempPoint = new FramePoint3D();

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final PIDSE3GainsReadOnly gains;

   private final BooleanProvider avoidFootRotations;
   private final BooleanProvider dampFootRotations;
   private final DoubleProvider footDamping;
   private final PIDSE3Gains localGains = new DefaultPIDSE3Gains();

   private final FootRotationDetector footRotationDetector;

   private final YoBoolean liftOff;
   private final YoDouble liftOffStartTime;
   private final YoDouble liftOffStartPitch;
   private final YoDouble liftOffFinalTime;
   private final YoDouble liftOffFinalPitch;
   private final YoDouble liftOffPitch;

   private final YoBoolean touchDown;
   private final YoDouble touchDownStartTime;
   private final YoDouble touchDownStartPitch;
   private final YoDouble touchDownFinalTime;
   private final YoDouble touchDownFinalPitch;
   private final YoDouble touchDownPitch;

   public SupportState(FootControlHelper footControlHelper, PIDSE3GainsReadOnly holdPositionGains, YoVariableRegistry parentRegistry)
   {
      super(footControlHelper);

      this.gains = holdPositionGains;

      String prefix = footControlHelper.getRobotSide().getLowerCaseName() + "Foot";
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      footSwitch = footControlHelper.getHighLevelHumanoidControllerToolbox().getFootSwitches().get(robotSide);
      controlFrame = new PoseReferenceFrame(prefix + "HoldPositionFrame", contactableFoot.getSoleFrame());
      desiredSoleFrame = new PoseReferenceFrame(prefix + "DesiredSoleFrame", worldFrame);

      footBarelyLoaded = new YoBoolean(prefix + "BarelyLoaded", registry);
      copOnEdge = new YoBoolean(prefix + "CopOnEdge", registry);
      footLoadThreshold = new YoDouble(prefix + "LoadThreshold", registry);
      footLoadThreshold.set(defaultFootLoadThreshold);

      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      rampUpAllowableToeLoadAfterContact = walkingControllerParameters.rampUpAllowableToeLoadAfterContact();
      toeLoadingDuration = new YoDouble(prefix + "ToeContactPointLoadingTime", registry);
      fullyLoadedMagnitude = new YoDouble(prefix + "FullyLoadedMagnitude", registry);
      toeLoadingDuration.set(walkingControllerParameters.getToeLoadingDuration());
      fullyLoadedMagnitude.set(walkingControllerParameters.getFullyLoadedToeForce());

      FullHumanoidRobotModel fullRobotModel = footControlHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);

      spatialFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);
      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);

      assumeCopOnEdge = new BooleanParameter(prefix + "AssumeCopOnEdge", registry, false);
      assumeFootBarelyLoaded = new BooleanParameter(prefix + "AssumeFootBarelyLoaded", registry, false);
      neverHoldRotation = new BooleanParameter(prefix + "NeverHoldRotation", registry, false);
      neverHoldPosition = new BooleanParameter(prefix + "NeverHoldPosition", registry, false);
      holdFootOrientationFlat = new BooleanParameter(prefix + "HoldFlatOrientation", registry, false);

      explorationHelper = new ExplorationHelper(contactableFoot, footControlHelper, prefix, registry);
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      requestFootholdExploration = new YoBoolean(prefix + "RequestFootholdExploration", registry);
      ExplorationParameters explorationParameters = footControlHelper.getExplorationParameters();
      if (walkingControllerParameters.createFootholdExplorationTools() && explorationParameters != null)
      {
         recoverTime = explorationParameters.getRecoverTime();
         timeBeforeExploring = explorationParameters.getTimeBeforeExploring();
      }
      else
      {
         recoverTime = new YoDouble(prefix + "RecoverTime", registry);
         timeBeforeExploring = new YoDouble(prefix + "TimeBeforeExploring", registry);
      }

      YoGraphicsListRegistry graphicsListRegistry = footControlHelper.getHighLevelHumanoidControllerToolbox().getYoGraphicsListRegistry();
      if (graphicsListRegistry != null)
      {
         frameViz = new YoGraphicReferenceFrame(controlFrame, registry, false, 0.2);
         graphicsListRegistry.registerYoGraphic(prefix + getClass().getSimpleName(), frameViz);
      }
      else
      {
         frameViz = null;
      }

      MovingReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
      double dt = controllerToolbox.getControlDT();
      footRotationDetector = new FootRotationDetector(robotSide, soleFrame, dt, registry, graphicsListRegistry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      avoidFootRotations = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "avoidFootRotations", registry, false);
      dampFootRotations = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "dampFootRotations", registry, false);
      footDamping = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "footDamping", registry, 0.0);

      liftOff = new YoBoolean(prefix + "LiftOff", registry);
      liftOffStartTime = new YoDouble(prefix + "LiftOffStartTime", registry);
      liftOffStartPitch = new YoDouble(prefix + "LiftOffStartPitch", registry);
      liftOffFinalTime = new YoDouble(prefix + "LiftOffFinalTime", registry);
      liftOffFinalPitch = new YoDouble(prefix + "LiftOffFinalPitch", registry);
      liftOffPitch = new YoDouble(prefix + "LiftOffPitch", registry);

      touchDown = new YoBoolean(prefix + "TouchDown", registry);
      touchDownStartTime = new YoDouble(prefix + "TouchDownStartTime", registry);
      touchDownStartPitch = new YoDouble(prefix + "TouchDownStartPitch", registry);
      touchDownFinalTime = new YoDouble(prefix + "TouchDownFinalTime", registry);
      touchDownFinalPitch = new YoDouble(prefix + "TouchDownFinalPitch", registry);
      touchDownPitch = new YoDouble(prefix + "TouchDownPitch", registry);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();
      FrameVector3D fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      controllerToolbox.setFootContactStateNormalContactVector(robotSide, fullyConstrainedNormalContactVector);

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      footBarelyLoaded.set(false);
      copOnEdge.set(false);
      liftOff.set(false);
      updateHoldPositionSetpoints();
   }

   @Override
   public void onExit()
   {
      super.onExit();
      footBarelyLoaded.set(false);
      copOnEdge.set(false);
      if (frameViz != null)
         frameViz.hide();
      explorationHelper.stopExploring();
      footRotationDetector.reset();

      liftOff.set(false);
      liftOffStartPitch.set(Double.NaN);
      liftOffFinalPitch.set(Double.NaN);
      liftOffStartTime.set(Double.NaN);
      liftOffFinalTime.set(Double.NaN);
      liftOffPitch.set(Double.NaN);

      touchDown.set(false);
      touchDownStartPitch.set(Double.NaN);
      touchDownFinalPitch.set(Double.NaN);
      touchDownStartTime.set(Double.NaN);
      touchDownFinalTime.set(Double.NaN);
      touchDownPitch.set(Double.NaN);

      desiredAngularVelocity.setToZero(worldFrame);
   }

   @Override
   public void doSpecificAction(double timeInState)
   {
      computeFootPolygon();
      controllerToolbox.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      // handle partial foothold detection
      boolean recoverTimeHasPassed = timeInState > recoverTime.getDoubleValue();
      boolean contactStateHasChanged = false;
      if (partialFootholdControlModule != null && recoverTimeHasPassed)
      {
         footSwitch.computeAndPackCoP(cop);
         partialFootholdControlModule.compute(desiredCoP, cop);
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactStateHasChanged = partialFootholdControlModule.applyShrunkPolygon(contactState);
         if (contactStateHasChanged)
            contactState.notifyContactStateHasChanged();
      }

      // foothold exploration
      boolean timeBeforeExploringHasPassed = timeInState > timeBeforeExploring.getDoubleValue();
      if (requestFootholdExploration.getBooleanValue() && timeBeforeExploringHasPassed)
      {
         explorationHelper.startExploring();
         requestFootholdExploration.set(false);
      }
      if (partialFootholdControlModule != null && !timeBeforeExploringHasPassed)
         partialFootholdControlModule.clearCoPGrid();
      explorationHelper.compute(timeInState, contactStateHasChanged);

      // toe contact point loading //// TODO: 6/5/17
      if (rampUpAllowableToeLoadAfterContact && timeInState < toeLoadingDuration.getDoubleValue())
      {

         double maxContactPointX = footPolygon.getMaxX();
         double minContactPointX = footPolygon.getMinX();

         double phaseInLoading = timeInState / toeLoadingDuration.getDoubleValue();
         double leadingToeMagnitude = InterpolationTools.linearInterpolate(0.0, fullyLoadedMagnitude.getDoubleValue(), phaseInLoading);
         YoPlaneContactState planeContactState = controllerToolbox.getFootContactState(robotSide);

         for (int i = 0; i < planeContactState.getTotalNumberOfContactPoints(); i++)
         {
            YoContactPoint contactPoint = planeContactState.getContactPoints().get(i);
            contactPoint.getPosition(tempPoint);
            double percentAlongFoot = (tempPoint.getX() - minContactPointX) / (maxContactPointX - minContactPointX);
            double contactPointMagnitude = InterpolationTools.linearInterpolate(fullyLoadedMagnitude.getDoubleValue(), leadingToeMagnitude, percentAlongFoot);

            planeContactState.setMaxContactPointNormalForce(contactPoint, contactPointMagnitude);
         }
      }
      else
      {
         YoPlaneContactState planeContactState = controllerToolbox.getFootContactState(robotSide);
         for (int i = 0; i < planeContactState.getTotalNumberOfContactPoints(); i++)
         {
            YoContactPoint contactPoint = planeContactState.getContactPoints().get(i);
            planeContactState.setMaxContactPointNormalForce(contactPoint, Double.POSITIVE_INFINITY);
         }
      }

      // determine foot state
      copOnEdge.set(footControlHelper.isCoPOnEdge());
      footBarelyLoaded.set(footSwitch.computeFootLoadPercentage() < footLoadThreshold.getDoubleValue());

      if (assumeCopOnEdge.getValue())
         copOnEdge.set(true);
      if (assumeFootBarelyLoaded.getValue())
         footBarelyLoaded.set(true);
      if (neverHoldRotation.getValue())
         copOnEdge.set(false);
      if (neverHoldPosition.getValue())
         footBarelyLoaded.set(false);

      updateHoldPositionSetpoints();

      localGains.set(gains);
      boolean dampingRotations = false;

      if (footRotationDetector.compute() && avoidFootRotations.getValue())
      {
         if (dampFootRotations.getValue())
         {
            PID3DGainsReadOnly orientationGains = gains.getOrientationGains();
            PID3DGains localOrientationGains = localGains.getOrientationGains();
            localOrientationGains.setProportionalGains(0.0, 0.0, orientationGains.getProportionalGains()[2]);
            localOrientationGains.setDerivativeGains(footDamping.getValue(), footDamping.getValue(), orientationGains.getDerivativeGains()[2]);
            dampingRotations = true;
         }
      }

      // update the control frame
      footSwitch.computeAndPackCoP(cop2d);
      if (cop2d.containsNaN())
         cop2d.setToZero(contactableFoot.getSoleFrame());
      framePosition.setIncludingFrame(cop2d, 0.0);
      frameOrientation.setToZero(contactableFoot.getSoleFrame());
      controlFrame.setPoseAndUpdate(framePosition, frameOrientation);

      // assemble acceleration command
      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.setToZero(bodyFixedFrame, rootBody.getBodyFixedFrame(), controlFrame);
      footAcceleration.setBodyFrame(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(controlFrame, footAcceleration);
      spatialAccelerationCommand.setWeights(angularWeight, linearWeight);

      // assemble feedback command
      bodyFixedControlledPose.setToZero(controlFrame);
      bodyFixedControlledPose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      desiredCopPosition.setIncludingFrame(cop2d, 0.0);
      desiredCopPosition.setReferenceFrame(desiredSoleFrame);
      desiredCopPosition.changeFrame(worldFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredCopPosition, desiredAngularVelocity, desiredLinearVelocity, desiredAngularAcceleration, desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);
      spatialFeedbackControlCommand.setGains(localGains);

      // set selection matrices
      accelerationSelectionMatrix.resetSelection();
      feedbackSelectionMatrix.resetSelection();

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      if (footBarelyLoaded.getBooleanValue())
      {
         isDirectionFeedbackControlled[3] = true; // control x position
         isDirectionFeedbackControlled[4] = true; // control y position
         isDirectionFeedbackControlled[2] = true; // control z orientation
      }
      if (copOnEdge.getBooleanValue() || dampingRotations)
      {
         isDirectionFeedbackControlled[0] = true; // control x orientation
         isDirectionFeedbackControlled[1] = true; // control y orientation
      }
      if (liftOff.getValue() || touchDown.getValue())
      {
         isDirectionFeedbackControlled[1] = true; // control y orientation
      }

      for (int i = dofs-1; i >= 0; i--)
      {
         if (isDirectionFeedbackControlled[i])
            accelerationSelectionMatrix.selectAxis(i, false);
         else
            feedbackSelectionMatrix.selectAxis(i, false);
      }

      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);

      // update visualization
      if (frameViz != null)
         frameViz.setToReferenceFrame(controlFrame);
   }


   private void computeFootPolygon()
   {
      ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
      footPolygon.clear(soleFrame);
      for (int i = 0; i < contactableFoot.getTotalNumberOfContactPoints(); i++)
         footPolygon.addVertex(contactableFoot.getContactPoints2d().get(i));
      footPolygon.update();
   }

   private void updateHoldPositionSetpoints()
   {
      footPosition.setToZero(contactableFoot.getSoleFrame());
      footOrientation.setToZero(contactableFoot.getSoleFrame());
      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);

      desiredPosition.checkReferenceFrameMatch(footPosition);
      desiredOrientation.checkReferenceFrameMatch(footOrientation);

      // The z component is always updated as it is never held in place
      if (footBarelyLoaded.getBooleanValue() && copOnEdge.getBooleanValue()) // => Holding X-Y-Yaw-Components (cuz barely loaded) and Pitch-Roll-Components (cuz CoP on edge)
      { // Only the z component is not held
         desiredPosition.setZ(footPosition.getZ());
      }
      else if (footBarelyLoaded.getBooleanValue()) // => Holding X-Y-Yaw-Components (cuz barely loaded)
      { // Update pitch and roll for when the CoP will get on the edge, and z as always
         desiredPosition.setZ(footPosition.getZ());
         desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), footOrientation.getPitch(), footOrientation.getRoll());
      }
      else if (copOnEdge.getBooleanValue()) // => Holding Pitch-Roll-Components (cuz CoP on edge)
      { // Update X-Y-Z and yaw for next time the foot will be barely loaded
         desiredPosition.set(footPosition);
         desiredOrientation.setYawPitchRoll(footOrientation.getYaw(), desiredOrientation.getPitch(), desiredOrientation.getRoll());
      }
      else // Not holding anything
      { // Update the full pose.
         desiredPosition.set(footPosition);
         desiredOrientation.set(footOrientation);
      }

      if (holdFootOrientationFlat.getValue())
         desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), 0.0, 0.0);

      if (liftOff.getValue())
      {
         double currentTime = controllerToolbox.getYoTime().getValue();
         double percent = (currentTime - liftOffStartTime.getValue()) / (liftOffFinalTime.getValue() - liftOffStartTime.getValue());
         percent = MathTools.clamp(percent, 0.0, 1.0);
         liftOffPitch.set(liftOffStartPitch.getValue() + (liftOffFinalPitch.getValue() - liftOffStartPitch.getValue()) * percent);
         desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), liftOffPitch.getValue(), desiredOrientation.getRoll());
         desiredAngularVelocity.set(0.0, (liftOffFinalPitch.getValue() - liftOffStartPitch.getValue()) / (liftOffFinalTime.getValue() - liftOffStartTime.getValue()), 0.0);
      }
      if (touchDown.getValue())
      {
         double currentTime = controllerToolbox.getYoTime().getValue();
         double percent = (currentTime - touchDownStartTime.getValue()) / (touchDownFinalTime.getValue() - touchDownStartTime.getValue());
         if (percent > 1.0)
         {
            finishTouchDown();
         }
         else
         {
            percent = MathTools.clamp(percent, 0.0, 1.0);
            touchDownPitch.set(touchDownStartPitch.getValue() + (touchDownFinalPitch.getValue() - touchDownStartPitch.getValue()) * percent);
            desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), touchDownPitch.getValue(), desiredOrientation.getRoll());
            desiredAngularVelocity.set(0.0, (touchDownFinalPitch.getValue() - touchDownStartPitch.getValue()) / (touchDownFinalTime.getValue() - touchDownStartTime.getValue()), 0.0);
         }
      }

      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);
   }

   public void liftOff(double finalPitchInSoleZUp, double duration)
   {
      if (liftOff.getValue() || touchDown.getValue())
         return;

      double currentTime = controllerToolbox.getYoTime().getValue();

      footOrientation.setToZero(contactableFoot.getSoleFrame());
      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide);
      footOrientation.changeFrame(soleZUpFrame);
      double currentPitch = footOrientation.getPitch();

      if (MathTools.epsilonEquals(finalPitchInSoleZUp, currentPitch, Math.toRadians(5.0)))
      {
         return;
      }

      liftOffStartPitch.set(currentPitch);
      liftOffFinalPitch.set(finalPitchInSoleZUp);
      liftOffStartTime.set(currentTime);
      liftOffFinalTime.set(currentTime + duration);
      liftOffPitch.set(currentPitch);

      if (finalPitchInSoleZUp > currentPitch)
      {
         enableToeContacts(controllerToolbox.getFootContactState(robotSide));
      }
      else
      {
         enableHeelContacts(controllerToolbox.getFootContactState(robotSide));
      }

      liftOff.set(true);
   }

   public void touchDown(double finalPitchInSoleZUp, double duration)
   {
      if (liftOff.getValue() || touchDown.getValue())
      {
         return;
      }

      double currentTime = controllerToolbox.getYoTime().getValue();

      footOrientation.setToZero(contactableFoot.getSoleFrame());
      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide);
      footOrientation.changeFrame(soleZUpFrame);
      double currentPitch = footOrientation.getPitch();

      if (MathTools.epsilonEquals(finalPitchInSoleZUp, currentPitch, Math.toRadians(5.0)))
      {
         finishTouchDown();
         return;
      }

      touchDownStartPitch.set(currentPitch);
      touchDownFinalPitch.set(finalPitchInSoleZUp);
      touchDownStartTime.set(currentTime);
      touchDownFinalTime.set(currentTime + duration);
      touchDownPitch.set(currentPitch);

      if (finalPitchInSoleZUp < currentPitch)
      {
         enableToeContacts(controllerToolbox.getFootContactState(robotSide));
      }
      else
      {
         enableHeelContacts(controllerToolbox.getFootContactState(robotSide));
      }

      touchDown.set(true);
   }

   private static void enableToeContacts(YoPlaneContactState contactState)
   {
      double maxX = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         maxX = Math.max(maxX, x);
      }
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         contactState.getContactPoints().get(i).setInContact(Precision.equals(x, maxX));
      }
      contactState.notifyContactStateHasChanged();
   }

   private static void enableHeelContacts(YoPlaneContactState contactState)
   {
      double minX = Double.POSITIVE_INFINITY;
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         minX = Math.min(minX, x);
      }
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         contactState.getContactPoints().get(i).setInContact(Precision.equals(x, minX));
      }
      contactState.notifyContactStateHasChanged();
   }

   private void finishTouchDown()
   {
      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         contactState.getContactPoints().get(i).setInContact(true);
      }
      contactState.notifyContactStateHasChanged();

      touchDown.set(false);
      touchDownStartPitch.set(Double.NaN);
      touchDownFinalPitch.set(Double.NaN);
      touchDownStartTime.set(Double.NaN);
      touchDownFinalTime.set(Double.NaN);
      touchDownPitch.set(Double.NaN);

      desiredAngularVelocity.setToZero(worldFrame);
   }

   public void requestFootholdExploration()
   {
      requestFootholdExploration.set(true);
      if (partialFootholdControlModule != null)
         partialFootholdControlModule.turnOnCropping();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandsList.clear();
      inverseDynamicsCommandsList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandsList.addCommand(explorationHelper.getCommand());

      return inverseDynamicsCommandsList;
   }


   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight = angularWeight;
      this.linearWeight = linearWeight;
   }

}
