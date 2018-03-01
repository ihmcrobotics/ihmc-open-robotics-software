package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * The unconstrained state is used if the foot is moved free in space without constrains. Depending on the type of trajectory
 * this can either be a movement along a straight line or (in case of walking) a swing motion.
 *
 * E.g. the MoveStraightState and the SwingState extend this class and implement the trajectory related methods.
 */
public abstract class AbstractUnconstrainedState extends AbstractFootControlState
{
   private static final boolean USE_ALL_LEG_JOINT_SWING_CORRECTOR = false;

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   protected final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;
   private final LegJointLimitAvoidanceControlModule legJointLimitAvoidanceControlModule;

   private final YoFramePoint yoDesiredPosition;
   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoBoolean yoSetDesiredAccelerationToZero;
   private final YoBoolean yoSetDesiredVelocityToZero;

   protected final YoBoolean scaleSecondaryJointWeights;
   protected final YoDouble secondaryJointWeightScale;

   private Vector3DReadOnly nominalAngularWeight;
   private Vector3DReadOnly nominalLinearWeight;
   private final YoFrameVector currentAngularWeight;
   private final YoFrameVector currentLinearWeight;

   private final ReferenceFrame ankleFrame;
   private final PoseReferenceFrame controlFrame;

   private final PIDSE3GainsReadOnly gains;

   public AbstractUnconstrainedState(ConstraintType constraintType, FootControlHelper footControlHelper, PIDSE3GainsReadOnly gains,
         YoVariableRegistry registry)
   {
      super(constraintType, footControlHelper);
      this.gains = gains;

      this.legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName() + FormattingTools.underscoredToCamelCase(constraintType.toString().toLowerCase(), true);
      yoDesiredLinearVelocity = new YoFrameVector(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      yoDesiredLinearVelocity.setToNaN();
      yoDesiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", worldFrame, registry);
      yoDesiredPosition.setToNaN();
      yoSetDesiredAccelerationToZero = new YoBoolean(namePrefix + "SetDesiredAccelerationToZero", registry);
      yoSetDesiredVelocityToZero = new YoBoolean(namePrefix + "SetDesiredVelocityToZero", registry);

      scaleSecondaryJointWeights = new YoBoolean(namePrefix + "ScaleSecondaryJointWeights", registry);
      secondaryJointWeightScale = new YoDouble(namePrefix + "SecondaryJointWeightScale", registry);
      secondaryJointWeightScale.set(1.0);

      currentAngularWeight = new YoFrameVector(namePrefix + "CurrentAngularWeight", worldFrame, registry);
      currentLinearWeight = new YoFrameVector(namePrefix + "CurrentLinearWeight", worldFrame, registry);

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
         legJointLimitAvoidanceControlModule = new LegJointLimitAvoidanceControlModule(namePrefix, registry, controllerToolbox, robotSide);
      else
         legJointLimitAvoidanceControlModule = null;

      ankleFrame = contactableFoot.getFrameAfterParentJoint();
      controlFrame = new PoseReferenceFrame("controlFrame", contactableFoot.getRigidBody().getBodyFixedFrame());

      spatialFeedbackControlCommand.set(rootBody, foot);
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);
      ReferenceFrame linearGainsFrame = footControlHelper.getHighLevelHumanoidControllerToolbox().getPelvisZUpFrame();
      spatialFeedbackControlCommand.setGainsFrames(null, linearGainsFrame);
      FramePose3D anklePoseInFoot = new FramePose3D(ankleFrame);
      anklePoseInFoot.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(anklePoseInFoot);
   }

   protected void changeControlFrame(FramePose3D controlFramePoseInEndEffector)
   {
      controlFramePoseInEndEffector.checkReferenceFrameMatch(contactableFoot.getRigidBody().getBodyFixedFrame());
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePoseInEndEffector);
      controlFrame.setPoseAndUpdate(controlFramePoseInEndEffector);
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.nominalAngularWeight = angularWeight;
      this.nominalLinearWeight = linearWeight;
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

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(true);
      }

      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
      contactState.notifyContactStateHasChanged();

      spatialFeedbackControlCommand.resetSecondaryTaskJointWeightScale();

      initializeTrajectory();
   }

   private final FramePoint3D desiredAnklePosition = new FramePoint3D();
   private final FramePose3D desiredPose = new FramePose3D();

   @Override
   public void doSpecificAction()
   {
      computeAndPackTrajectory();

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
      {
         legJointLimitAvoidanceControlModule.correctSwingFootTrajectory(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration);
      }

      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         desiredPose.setIncludingFrame(desiredPosition, desiredOrientation);
         changeDesiredPoseBodyFrame(controlFrame, ankleFrame, desiredPose);
         desiredAnklePosition.setIncludingFrame(desiredPose.getPosition());

         legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectory(desiredAnklePosition, desiredLinearVelocity, desiredLinearAcceleration);

         desiredPose.setPosition(desiredAnklePosition);
         changeDesiredPoseBodyFrame(ankleFrame, controlFrame, desiredPose);
         desiredPosition.setIncludingFrame(desiredPose.getPosition());
      }

      if (yoSetDesiredVelocityToZero.getBooleanValue())
      {
         desiredLinearVelocity.setToZero();
      }

      if (yoSetDesiredAccelerationToZero.getBooleanValue())
      {
         desiredLinearAcceleration.setToZero();
      }

      computeCurrentWeights(nominalAngularWeight, nominalLinearWeight, currentAngularWeight, currentLinearWeight);

      spatialFeedbackControlCommand.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(currentAngularWeight, currentLinearWeight);
      spatialFeedbackControlCommand.setScaleSecondaryTaskJointWeight(scaleSecondaryJointWeights.getBooleanValue(), secondaryJointWeightScale.getDoubleValue());
      spatialFeedbackControlCommand.setGains(gains);

      yoDesiredPosition.setAndMatchFrame(desiredPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(desiredLinearVelocity);
   }

   /**
    * Overwrite this for weight scheduling.
    */
   protected void computeCurrentWeights(Vector3DReadOnly nominalAngularWeight, Vector3DReadOnly nominalLinearWeight,
                                        Vector3DBasics currentAngularWeightToPack, Vector3DBasics currentLinearWeightToPack)
   {
      currentAngularWeightToPack.set(nominalAngularWeight);
      currentLinearWeightToPack.set(nominalLinearWeight);
   }

   private final RigidBodyTransform oldBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewBodyFrameToOldBodyFrame = new RigidBodyTransform();

   private void changeDesiredPoseBodyFrame(ReferenceFrame oldBodyFrame, ReferenceFrame newBodyFrame, FramePose3D framePoseToModify)
   {
      if (oldBodyFrame == newBodyFrame)
         return;

      framePoseToModify.get(oldBodyFrameDesiredTransform);
      newBodyFrame.getTransformToDesiredFrame(transformFromNewBodyFrameToOldBodyFrame, oldBodyFrame);
      newBodyFrameDesiredTransform.set(oldBodyFrameDesiredTransform);
      newBodyFrameDesiredTransform.multiply(transformFromNewBodyFrameToOldBodyFrame);
      framePoseToModify.set(newBodyFrameDesiredTransform);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      yoDesiredPosition.setToNaN();
      yoDesiredLinearVelocity.setToNaN();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }
}
