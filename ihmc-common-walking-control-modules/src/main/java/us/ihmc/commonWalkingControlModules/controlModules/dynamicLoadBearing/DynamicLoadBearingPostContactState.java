package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.LoadBearingParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DynamicLoadBearingPostContactState implements DynamicLoadBearingState
{
   private static final double COEFFICIENT_OF_FRICTION = 0.5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3D zeroWorld = new FrameVector3D();

   /* Controller Commands */
   private final RigidBodyBasics bodyToControl;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final MutableBoolean hasContactChanged;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
   private final PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();

   private final SpatialAcceleration bodyAcceleration;
   private final PoseReferenceFrame desiredContactFrameFixedInWorld;

   /* Control gains, weights and axis selection */
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

   private final FramePoint3D contactPointInBody = new FramePoint3D();
   private final FramePoint3D bracingContactPoint = new FramePoint3D();
   private final FrameVector3D bracingNormal = new FrameVector3D();

   private final OneDoFJointBasics[] jointPath;

   private final FramePoint3D currentContactPointInWorld = new FramePoint3D();
   private final FramePoint3D shoulderPointInWorld = new FramePoint3D();

   public DynamicLoadBearingPostContactState(LoadBearingParameters loadBearingParameters,
                                             RigidBodyBasics bodyToControl,
                                             RigidBodyBasics baseBody,
                                             RigidBodyBasics elevator,
                                             ReferenceFrame controlFrame,
                                             MutableBoolean hasContactChanged,
                                             YoRegistry parentRegistry)
   {
      this.bodyToControl = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.hasContactChanged = hasContactChanged;

      String prefix = bodyToControl.getParentJoint().getName().split("_")[0];
      JointBasics shoulderJoint = MultiBodySystemTools.findJoint(baseBody, prefix + "_SHOULDER_Y");
      jointPath = MultiBodySystemTools.createOneDoFJointPath(shoulderJoint.getPredecessor(), bodyToControl);

      bracingContactPoint.setToZero(controlFrame);
      contactPointInBody.setToZero(controlFrame);
      contactPointInBody.changeFrame(bodyToControl.getBodyFixedFrame());

      String bodyName = bodyToControl.getName();
      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setPrimaryBase(baseBody);

      desiredContactFrameFixedInWorld = new PoseReferenceFrame("desiredContactFrame" + bodyName, ReferenceFrame.getWorldFrame());
      bodyAcceleration = new SpatialAcceleration(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);

      feedbackGains = new DefaultYoPIDSE3Gains("LoadBearing", GainCoupling.XY, false, parentRegistry);
      configureGains();

      bodyBarelyLoaded = new GlitchFilteredYoBoolean(bodyName + "BarelyLoaded", parentRegistry, 60);
      yoControllerDesiredForce = new YoFrameVector3D(bodyName + "DesiredForce", ReferenceFrame.getWorldFrame(), parentRegistry);
   }

   public void setBracingSurface(Vector3DReadOnly bracingNormal)
   {
      this.bracingNormal.set(bracingNormal);
   }

   /**
    * Assumes when this state is entered that the hand is in contact with the surface, to the best of the estimator's knowledge.
    * For simulation, this is just done with just distance to the planar region.
    * For real robot, maybe augment with some proprioceptive knowledge. Or maybe the map is good enough.
    *
    * Orientation is not controlled during pre-contact, but should be here (should it be?)
    */
   @Override
   public void onEntry()
   {
      hasContactChanged.setValue(true);
   }

   @Override
   public void doAction(double timeInState)
   {
      currentContactPointInWorld.setMatchingFrame(contactPointInBody);

      if (controllerCoreOutput.getDesiredExternalWrench(controllerDesiredWrench, bodyToControl))
      { // Determine load status from controller core desired, assume it tracks
         double desiredForceLoadMagnitudeSquared = controllerDesiredWrench.getLinearPart().normSquared();

         double forceThreshold = 12.0;
         bodyBarelyLoaded.update(desiredForceLoadMagnitudeSquared < MathTools.square(forceThreshold));

         controllerDesiredForce.setIncludingFrame(controllerDesiredWrench.getReferenceFrame(), controllerDesiredWrench.getLinearPart());
         controllerDesiredForce.changeFrame(ReferenceFrame.getWorldFrame());
         yoControllerDesiredForce.set(controllerDesiredForce);
      }
      else
      { // If no desired wrench, set to barely loaded
         bodyBarelyLoaded.set(true);
      }

      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(COEFFICIENT_OF_FRICTION);
      planeContactStateCommand.setContactNormal(bracingNormal);
      planeContactStateCommand.addPointInContact(contactPointInBody);
      planeContactStateCommand.setHasContactStateChanged(false);

      bodyAcceleration.setToZero(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(desiredContactFrameFixedInWorld, bodyAcceleration);
      spatialAccelerationSelectionMatrix.getAngularPart().clearSelection();
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.X.ordinal(), !bodyBarelyLoaded.getValue());
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Y.ordinal(), !bodyBarelyLoaded.getValue());
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Z.ordinal(), true);
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);
      spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(50.0); // TODO parameterize

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

   }

   @Override
   public void onExit(double timeInState)
   {
      hasContactChanged.setValue(true);
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
      // Maybe at point feedback later, but for now just use spatial acceleration command

      return null;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return null;
   }

   @Override
   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setHasContactStateChanged(true);
      return planeContactStateCommand;
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

   @Override
   public boolean isDone(double timeInState)
   {
      // Check if near reachability limit
      shoulderPointInWorld.setToZero(jointPath[0].getFrameBeforeJoint());
      shoulderPointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      double distanceThreshold = 0.76;
      if (shoulderPointInWorld.distanceSquared(currentContactPointInWorld) > distanceThreshold * distanceThreshold)
         return true;

      // Check if joints are near a joint limit
      for (int i = 0; i < jointPath.length; i++)
      {
         if (jointIsNearLimit(jointPath[i]))
            return true;
      }

      // Somehow trigger go-home message if true, or maybe that should be done a layer up

      return false;
   }

   private static boolean jointIsNearLimit(OneDoFJointBasics joint)
   {
      double q = joint.getQ();
      double qMin = joint.getJointLimitLower();
      double qMax = joint.getJointLimitUpper();
      double epsilon = 0.12;

      return q < qMin + epsilon || q > qMax - epsilon;
   }

   public void updateWholeBodyContactState(WholeBodyContactState wholeBodyContactStateToUpdate)
   {
      wholeBodyContactStateToUpdate.addContactPoints(planeContactStateCommand);
   }

   public void packContactData(RecyclingArrayList<Point3D> contactPointList, Vector3DBasics contactNormalToPack)
   {
      contactPointList.add().set(contactPointInBody);
      contactNormalToPack.set(bracingNormal);
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
