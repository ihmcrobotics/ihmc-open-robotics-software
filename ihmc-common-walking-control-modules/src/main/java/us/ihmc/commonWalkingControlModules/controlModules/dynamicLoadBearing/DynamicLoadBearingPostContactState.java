package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.LoadBearingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyOrientationControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPositionControlHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
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
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
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
import us.ihmc.yoVariables.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.Supplier;

public class DynamicLoadBearingPostContactState implements DynamicLoadBearingState
{
   // Debug variables for toggling various objectives
   private static final double COEFFICIENT_OF_FRICTION = 0.5;
   private static final boolean ENABLE_CONTACT = true;
   private static final boolean ENABLE_ZERO_ACCELERATION = true;
   private static final boolean ENABLE_POINT_FEEDBACK = true;
   private static final boolean ENABLE_ORIENTATION_FEEDBACK = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   /* Controller Commands */
   private final RigidBodyBasics bodyToControl;
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   /* Control gains, weights and axis selection */
   private final LoadBearingParameters loadBearingParameters;
   private final BooleanProvider useImpedanceControl;
   private final Vector3DReadOnly linearWeight = new Vector3D(50.0, 50.0, 50.0);
   private final Vector3DReadOnly angularWeight = new Vector3D(5.0, 5.0, 5.0);
   private final DefaultYoPIDSE3Gains feedbackGains;
   private final SelectionMatrix3D positionFeedbackSelectionMatrix = new SelectionMatrix3D();
   private final SelectionMatrix6D spatialAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final double nominalRhoWeight;

   /* Hand load status */
   private final GlitchFilteredYoBoolean bodyBarelyLoaded;
   private ControllerCoreOutputReadOnly controllerCoreOutput;
   private final Supplier<SpatialVectorReadOnly> estimatedExternalWrenchSupplier;
   private final Wrench controllerDesiredWrench = new Wrench();
   private final FrameVector3D controllerDesiredForce = new FrameVector3D();
   private final YoFrameVector3D yoControllerDesiredForce;
   private final FrameVector3D estimatedExternalForce = new FrameVector3D();
   private final FrameVector3D estimatedExternalForceInContactFrame = new FrameVector3D();
   private final YoFrameVector3D yoEstimatedExternalForce;
   private final YoDouble yoEstimatedNormalForce;
   private final YoDouble yoDesiredNormalForce;
   private final YoDouble yoNormalForceError;
   private final YoDouble yoNormalPositionOffset;
   private final YoDouble yoNormalVelocityCommand;

   /* Reference frames and contact data */
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final FramePoint3D contactPointInBody = new FramePoint3D();
   private final FramePose3D desiredContactPoseWorld = new FramePose3D();
   private final PoseReferenceFrame desiredContactFrameFixedInWorld;
   private final OneDoFJointBasics[] jointPath;
   private final FramePoint3D shoulderPointInWorld = new FramePoint3D();

   /* Yo-Contact frame components */
   private final YoFrameVector3D contactNormal;
   private final YoFramePoint3D currentContactPointInWorld;
   private final YoFramePoint3D yoDesiredContactPosition;
   private final YoFrameQuaternion yoDesiredContactOrientation;

   /* Trajectory handlers */
   private final RigidBodyPositionControlHelper positionControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;
   private final SO3TrajectoryControllerCommand orientationTrajectoryCommand = new SO3TrajectoryControllerCommand();

   private final SpatialAcceleration bodyAcceleration;

   /* Error measurement to detect slipping */
   private final FrameVector3D positionError = new FrameVector3D();
   private final YoFrameVector3D yoPositionError;
   private final FramePoint3D pointFeedbackDesiredPosition = new FramePoint3D();
   private final FrameVector3D pointFeedbackDesiredVelocity = new FrameVector3D();
   private final FrameVector3D pointFeedbackOffset = new FrameVector3D();
   private double lastTimeInState = 0.0;

   /* Flag for notifying contact change */
   private final MutableBoolean hasAddedContacts;
   private final MutableBoolean hasRemovedContacts;

   private Runnable onTouchdownCallback = () -> {};

   public DynamicLoadBearingPostContactState(RigidBodyBasics bodyToControl,
                                             RigidBodyBasics baseBody,
                                             RigidBodyBasics elevator,
                                             ReferenceFrame controlFrame,
                                             RigidBodyPositionControlHelper positionControlHelper,
                                             RigidBodyOrientationControlHelper orientationControlHelper,
                                             LoadBearingParameters loadBearingParameters,
                                             double nominalRhoWeight,
                                             Supplier<SpatialVectorReadOnly> estimatedExternalWrenchSupplier,
                                             MutableBoolean hasAddedContacts,
                                             MutableBoolean hasRemovedContacts,
                                             YoRegistry registry)
   {
      this.bodyToControl = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.loadBearingParameters = loadBearingParameters;
      this.useImpedanceControl = loadBearingParameters.getUseImpedanceControl();
      this.estimatedExternalWrenchSupplier = estimatedExternalWrenchSupplier;
      this.nominalRhoWeight = nominalRhoWeight;
      this.hasAddedContacts = hasAddedContacts;
      this.hasRemovedContacts = hasRemovedContacts;

      String bodyName = bodyToControl.getName();

      desiredContactFrameFixedInWorld = new PoseReferenceFrame("desiredContactFrame" + bodyName, ReferenceFrame.getWorldFrame());
      bodyAcceleration = new SpatialAcceleration(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);

      pointFeedbackControlCommand.set(elevator, bodyToControl);
      pointFeedbackControlCommand.setPrimaryBase(baseBody);

      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setPrimaryBase(baseBody);

      feedbackGains = new DefaultYoPIDSE3Gains("LoadBearing", GainCoupling.XY, false, registry);
      configureGains();

      contactNormal = new YoFrameVector3D(bodyName + "ContactNormal", worldFrame, registry);
      currentContactPointInWorld = new YoFramePoint3D(bodyName + "currentContactPoint", worldFrame, registry);
      yoDesiredContactPosition = new YoFramePoint3D(bodyName + "_DesiredContactPosition", worldFrame, registry);
      yoDesiredContactOrientation = new YoFrameQuaternion(bodyName + "_DesiredContactOrientation", worldFrame, registry);

      bodyBarelyLoaded = new GlitchFilteredYoBoolean(bodyName + "BarelyLoaded", registry, 20);
      yoControllerDesiredForce = new YoFrameVector3D(bodyName + "DesiredForce", ReferenceFrame.getWorldFrame(), registry);
      yoEstimatedExternalForce = new YoFrameVector3D(bodyName + "EstimatedExternalForce", ReferenceFrame.getWorldFrame(), registry);
      yoEstimatedNormalForce = new YoDouble(bodyName + "EstimatedNormalForce", registry);
      yoDesiredNormalForce = new YoDouble(bodyName + "DesiredNormalForce", registry);
      yoNormalForceError = new YoDouble(bodyName + "NormalForceError", registry);
      yoNormalPositionOffset = new YoDouble(bodyName + "NormalPositionOffset", registry);
      yoNormalVelocityCommand = new YoDouble(bodyName + "NormalVelocityCommand", registry);

      yoPositionError = new YoFrameVector3D(bodyName + "PositionError", desiredContactFrameFixedInWorld, registry);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      this.positionControlHelper = positionControlHelper;
      this.orientationControlHelper = orientationControlHelper;

      String prefix = bodyToControl.getParentJoint().getName().split("_")[0];
      JointBasics shoulderJoint = MultiBodySystemTools.findJoint(baseBody, prefix + "_SHOULDER_Y");
      jointPath = MultiBodySystemTools.createOneDoFJointPath(shoulderJoint.getPredecessor(), bodyToControl);

      contactPointInBody.setToZero(controlFrame);
      contactPointInBody.changeFrame(bodyToControl.getBodyFixedFrame());
   }

   private void configureGains()
   {
      double kpXYPosition = 100.0;
      double kpZPosition = loadBearingParameters.getNormalPositionTrackingStiffness();
      double zetaXYPosition = 1.0;
      double kdXYPosition = GainCalculator.computeDerivativeGain(kpXYPosition, zetaXYPosition);
      double kdZ = GainCalculator.computeDerivativeGain(kpZPosition, loadBearingParameters.getNormalPositionTrackingDampingRatio());
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

      // bias towards nominal, but heavily limit feedback
      feedbackGains.getOrientationGains().setMaxProportionalError(Math.toRadians(12.0));
   }

   @Override
   public void doAction(double timeInState)
   {
      currentContactPointInWorld.setMatchingFrame(contactPointInBody);

      if (controllerCoreOutput != null && controllerCoreOutput.getDesiredExternalWrench(controllerDesiredWrench, bodyToControl))
      {
         controllerDesiredForce.setIncludingFrame(controllerDesiredWrench.getReferenceFrame(), controllerDesiredWrench.getLinearPart());
         controllerDesiredForce.changeFrame(ReferenceFrame.getWorldFrame());
         yoControllerDesiredForce.set(controllerDesiredForce);
      }
      else
      {
         controllerDesiredForce.setToZero(worldFrame);
         yoControllerDesiredForce.setToNaN();
      }

      double normalForceEstimate = updateEstimatedNormalForce();
      if (useImpedanceControl.getValue())
      {
         bodyBarelyLoaded.update(Math.abs(normalForceEstimate) < loadBearingParameters.getNormalForceThresholdForLoaded());
      }
      else if (controllerCoreOutput != null && controllerCoreOutput.getDesiredExternalWrench(controllerDesiredWrench, bodyToControl))
      {
         double desiredForceLoadMagnitudeSquared = controllerDesiredWrench.getLinearPart().normSquared();
         bodyBarelyLoaded.update(desiredForceLoadMagnitudeSquared < MathTools.square(loadBearingParameters.getNormalForceThresholdForLoaded()));
      }
      else
      {
         bodyBarelyLoaded.set(true);
      }

      // assemble contact command
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(COEFFICIENT_OF_FRICTION);
      planeContactStateCommand.setContactNormal(contactNormal);
      planeContactStateCommand.addPointInContact(contactPointInBody);
      planeContactStateCommand.setHasContactStateChanged(false);

//      double alphaLoaded = EuclidCoreTools.clamp(timeInState / loadBearingParameters.getHandLoadDuration(), 0.0, 1.0);
//      double rhoWeightInterpolated = EuclidCoreTools.interpolate(RHO_WEIGHT_INITIAL, nominalRhoWeight, alphaLoaded);

//      for (int i = 0; i < planeContactStateCommand.getNumberOfContactPoints(); i++)
//      {
//         planeContactStateCommand.setRhoWeight(i, rhoWeightInterpolated);
//      }

      // assemble zero acceleration command
      bodyAcceleration.setToZero(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(desiredContactFrameFixedInWorld, bodyAcceleration);
      spatialAccelerationSelectionMatrix.getAngularPart().clearSelection();
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.X.ordinal(), !bodyBarelyLoaded.getValue());
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Y.ordinal(), !bodyBarelyLoaded.getValue());
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Z.ordinal(), !useImpedanceControl.getValue());
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);
      spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(linearWeight);

      // record contact point tracking error
      positionError.sub(currentContactPointInWorld, desiredContactPoseWorld.getPosition());
      yoPositionError.setMatchingFrame(positionError);

      // assemble spatial feedback command
      updatePointFeedbackCommand(timeInState, normalForceEstimate);

      orientationControlHelper.doAction(timeInState);
   }

   public void setBracingSurface(Vector3DReadOnly contactNormalInWorldFrame)
   {
      // Initialize contact point and contact normal
      this.contactNormal.set(contactNormalInWorldFrame);
   }

   @Override
   public void onEntry()
   {
      // Set to barely loaded during first tick
      bodyBarelyLoaded.set(true);
      lastTimeInState = 0.0;
      yoEstimatedExternalForce.setToNaN();
      yoEstimatedNormalForce.set(0.0);
      yoDesiredNormalForce.set(loadBearingParameters.getDesiredNormalForce());
      yoNormalForceError.set(0.0);
      yoNormalPositionOffset.set(0.0);
      yoNormalVelocityCommand.set(0.0);

      // Reset orientation trajectory
      orientationControlHelper.holdCurrent();

      // Compute desired contact pose, which is static in world, has an origin at the contact point and has Z pointing parallel to the contact normal
      desiredContactPoseWorld.setReferenceFrame(ReferenceFrame.getWorldFrame());
      desiredContactPoseWorld.getPosition().setMatchingFrame(this.contactPointInBody);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, contactNormal, desiredContactPoseWorld.getOrientation());
      desiredContactFrameFixedInWorld.setPoseAndUpdate(desiredContactPoseWorld);

      // Update yovariables
      yoDesiredContactPosition.set(desiredContactPoseWorld.getPosition());
      yoDesiredContactOrientation.set(desiredContactPoseWorld.getOrientation());

      hasAddedContacts.setTrue();

      onTouchdownCallback.run();
   }

   @Override
   public void onExit(double timeInState)
   {
      yoDesiredContactPosition.setToNaN();
      yoDesiredContactOrientation.setToNaN();
      currentContactPointInWorld.setToNaN();
      yoEstimatedExternalForce.setToNaN();
      yoEstimatedNormalForce.setToNaN();
      yoDesiredNormalForce.setToNaN();
      yoNormalForceError.setToNaN();
      yoNormalPositionOffset.setToNaN();
      yoNormalVelocityCommand.setToNaN();

      // hide graphics from taskspace controller
      positionControlHelper.getYoDesiredPosition().setToNaN();
      positionControlHelper.getYoCurrentPosition().setToNaN();

      orientationControlHelper.clear();

      hasRemovedContacts.setTrue();
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

      if (ENABLE_POINT_FEEDBACK && (bodyBarelyLoaded.getValue() || useImpedanceControl.getValue()))
      {
         pointFeedbackControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
         feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
      }
      if (ENABLE_ORIENTATION_FEEDBACK)
      {
         OrientationFeedbackControlCommand orientationFeedbackCommand = orientationControlHelper.getFeedbackControlCommand();
         orientationFeedbackCommand.setWeightsForSolver(angularWeight);
         orientationFeedbackCommand.setGains(feedbackGains.getOrientationGains());
         orientationFeedbackCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
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
   public boolean isDone(double time)
   {
      double positionErrorXYSquared = EuclidCoreTools.normSquared(yoPositionError.getX(), yoPositionError.getY());
      double linearTrackingSlipThresholdSquared = EuclidCoreTools.square(loadBearingParameters.getLinearTrackingSlipThreshold());
      if (positionErrorXYSquared > linearTrackingSlipThresholdSquared)
         return true;

      // Check if near reachability limit
      shoulderPointInWorld.setToZero(jointPath[1].getFrameBeforeJoint());
      shoulderPointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      double distanceThreshold = 0.7;
      if (shoulderPointInWorld.distanceSquared(currentContactPointInWorld) > distanceThreshold * distanceThreshold)
         return true;

      // Check if joints are near a joint limit
      for (int i = 0; i < jointPath.length; i++)
      {
         if (jointIsNearLimit(jointPath[i]))
            return true;
      }

      return false;
   }

   public void setOnTouchdownCallback(Runnable onTouchdownCallback)
   {
      this.onTouchdownCallback = onTouchdownCallback;
   }

   private void updatePointFeedbackCommand(double timeInState, double normalForceEstimate)
   {
      pointFeedbackDesiredPosition.setIncludingFrame(desiredContactPoseWorld.getPosition());
      pointFeedbackDesiredVelocity.setToZero(worldFrame);

      if (useImpedanceControl.getValue())
      {
         updateNormalAxisImpedanceReference(timeInState, normalForceEstimate);
      }
      else
      {
         yoDesiredNormalForce.set(loadBearingParameters.getDesiredNormalForce());
         yoNormalForceError.set(0.0);
         yoNormalVelocityCommand.set(0.0);
         yoNormalPositionOffset.set(0.0);
         lastTimeInState = timeInState;
      }

      pointFeedbackControlCommand.setBodyFixedPointToControl(contactPointInBody);
      pointFeedbackControlCommand.setGainsFrame(desiredContactFrameFixedInWorld);
      pointFeedbackControlCommand.setInverseDynamics(pointFeedbackDesiredPosition, pointFeedbackDesiredVelocity, null);
      pointFeedbackControlCommand.setWeightsForSolver(linearWeight);

      double xyKp = bodyBarelyLoaded.getValue() ? loadBearingParameters.getHoldPositionStiffness() : 0.0;
      double xyKd = GainCalculator.computeDerivativeGain(xyKp, loadBearingParameters.getHoldPositionDampingRatio());
      double zKp = useImpedanceControl.getValue() ? loadBearingParameters.getNormalPositionTrackingStiffness() : 0.0;
      double zKd = GainCalculator.computeDerivativeGain(zKp, loadBearingParameters.getNormalPositionTrackingDampingRatio());
      pointFeedbackControlCommand.getGains().setProportionalGains(xyKp, xyKp, zKp);
      pointFeedbackControlCommand.getGains().setDerivativeGains(xyKd, xyKd, zKd);

      positionFeedbackSelectionMatrix.clearSelection();
      positionFeedbackSelectionMatrix.selectAxis(Axis3D.X.ordinal(), bodyBarelyLoaded.getValue());
      positionFeedbackSelectionMatrix.selectAxis(Axis3D.Y.ordinal(), bodyBarelyLoaded.getValue());
      positionFeedbackSelectionMatrix.selectAxis(Axis3D.Z.ordinal(), useImpedanceControl.getValue());
      pointFeedbackControlCommand.setSelectionMatrix(positionFeedbackSelectionMatrix);
   }

   private void updateNormalAxisImpedanceReference(double timeInState, double normalForceEstimate)
   {
      double dt = Math.max(0.0, timeInState - lastTimeInState);
      lastTimeInState = timeInState;

      double desiredNormalForce = loadBearingParameters.getDesiredNormalForce();
      double normalForceError = desiredNormalForce - normalForceEstimate;
      double normalVelocityCommand = -loadBearingParameters.getNormalForceAdmittanceGain() * normalForceError
                                    - loadBearingParameters.getNormalForcePositionLeakRate() * yoNormalPositionOffset.getDoubleValue();
      normalVelocityCommand = MathTools.clamp(normalVelocityCommand, -loadBearingParameters.getMaxNormalVelocity(), loadBearingParameters.getMaxNormalVelocity());

      double normalPositionOffset = yoNormalPositionOffset.getDoubleValue() + dt * normalVelocityCommand;
      normalPositionOffset = MathTools.clamp(normalPositionOffset,
                                             -loadBearingParameters.getMaxNormalPositionOffset(),
                                             loadBearingParameters.getMaxNormalPositionOffset());

      yoDesiredNormalForce.set(desiredNormalForce);
      yoNormalForceError.set(normalForceError);
      yoNormalVelocityCommand.set(normalVelocityCommand);
      yoNormalPositionOffset.set(normalPositionOffset);

      pointFeedbackOffset.setIncludingFrame(contactNormal);
      pointFeedbackOffset.scale(normalPositionOffset);
      pointFeedbackDesiredPosition.add(pointFeedbackOffset);

      pointFeedbackDesiredVelocity.setIncludingFrame(contactNormal);
      pointFeedbackDesiredVelocity.scale(normalVelocityCommand);
   }

   private double updateEstimatedNormalForce()
   {
      double controllerDesiredNormalForce = 0.0;
      if (!yoControllerDesiredForce.containsNaN())
      {
         estimatedExternalForceInContactFrame.setIncludingFrame(controllerDesiredForce);
         estimatedExternalForceInContactFrame.changeFrame(desiredContactFrameFixedInWorld);
         controllerDesiredNormalForce = estimatedExternalForceInContactFrame.getZ();
      }

      if (estimatedExternalWrenchSupplier == null)
      {
         yoEstimatedExternalForce.setToNaN();
         yoEstimatedNormalForce.set(controllerDesiredNormalForce);
         return controllerDesiredNormalForce;
      }

      SpatialVectorReadOnly estimatedExternalWrench = estimatedExternalWrenchSupplier.get();
      if (estimatedExternalWrench == null)
      {
         yoEstimatedExternalForce.setToNaN();
         yoEstimatedNormalForce.set(controllerDesiredNormalForce);
         return controllerDesiredNormalForce;
      }

      estimatedExternalForce.setIncludingFrame(estimatedExternalWrench.getLinearPart());
      estimatedExternalForce.changeFrame(worldFrame);
      yoEstimatedExternalForce.set(estimatedExternalForce);

      estimatedExternalForceInContactFrame.setIncludingFrame(estimatedExternalForce);
      estimatedExternalForceInContactFrame.changeFrame(desiredContactFrameFixedInWorld);
      double estimatedNormalForce = estimatedExternalForceInContactFrame.getZ();
      yoEstimatedNormalForce.set(estimatedNormalForce);
      return estimatedNormalForce;
   }

   private static boolean jointIsNearLimit(OneDoFJointBasics joint)
   {
      double q = joint.getQ();
      double qMin = joint.getJointLimitLower();
      double qMax = joint.getJointLimitUpper();
      double epsilon = 0.1;

      return q < qMin + epsilon || q > qMax - epsilon;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
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
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(bodyToControl.getName() + "EstimatedExternalForce",
                                                                    currentContactPointInWorld,
                                                                    yoEstimatedExternalForce,
                                                                    0.0075,
                                                                    ColorDefinitions.Blue()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(bodyToControl.getName() + "ContactControlFrame", yoDesiredContactPosition,
                                                                               yoDesiredContactOrientation,
                                                                               0.12,
                                                                               ColorDefinitions.LightGray()));
      return group;
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
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

   public void packContactData(RecyclingArrayList<Point3D> contactPointList, Vector3DBasics contactNormalToPack)
   {
      contactPointList.add().set(contactPointInBody);
      contactNormalToPack.set(contactNormal);
   }
}
