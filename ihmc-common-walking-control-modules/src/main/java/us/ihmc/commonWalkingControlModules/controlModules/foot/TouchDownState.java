package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.geometry.FrameLineSegment;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.tools.lists.ListSorter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TouchDownState extends AbstractFootControlState
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand zeroAccelerationCommand = new SpatialAccelerationCommand();

   private final SelectionMatrix6D footAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackControlSelectionMatrix = new SelectionMatrix6D();

   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectory;
   private final HermiteCurveBasedOrientationTrajectoryGenerator orientationTrajectory;
   private final FootContactStateInterpolator footContactPointInterpolator;

   private final FrameLineSegment contactLine = new FrameLineSegment();

   private final FramePoint3D currentFootPosition = new FramePoint3D();

   private final YoFramePoint initialPosition;
   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameVector desiredInitialTouchdownVelocity;
   private final YoFrameVector desiredInitialTouchdownAcceleration;

   private final FramePoint3D contactPointPosition = new FramePoint3D();

   private final YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
   private final List<YoContactPoint> contactPoints = new ArrayList<>(contactState.getContactPoints());

   private final YoBoolean usePointContact;
   private final YoBoolean inContact;
   private final YoBoolean rampingContactNormals;
   private final YoDouble timeInContact;
   private final YoDouble contactStartTime;
   private final YoDouble rampingContactNormalsStartTime;
   private final YoDouble desiredTouchdownDuration;
   private final YoDouble percentageOfTouchdownDurationToInitiateRamping;
   private final YoDouble thresholdOfFootLoadingToAssumeInitialContact;
   private final YoDouble contactNormalMaximumForce;

   private final GroundContactReferenceFrame groundContactFrame;
   private final FootSwitchInterface footSwitch;
   private final FramePoint2D sensedCoP = new FramePoint2D();
   private final ReferenceFrame soleFrame;
   
   private final ReferenceFrame controlFrame;
   private final MovingReferenceFrame footBodyFixedFrame;
   private final FramePose controlFramePose = new FramePose();
   
   private final RigidBodyTransform tempRigidBodyTransform = new RigidBodyTransform();
   private final YoPIDSE3Gains inContactFeedbackGains;
   private final YoPIDSE3Gains swingFootControlGains;

   public TouchDownState(FootControlHelper footControlHelper, YoPIDSE3Gains swingFootControlGains, YoFrameVector yoTouchdownVelocity,
         YoFrameVector yoTouchdownAcceleration, YoVariableRegistry parentRegistry)
   {
      super(ConstraintType.TOUCHDOWN, footControlHelper);

      this.swingFootControlGains = swingFootControlGains;
      desiredInitialTouchdownVelocity = yoTouchdownVelocity;
      desiredInitialTouchdownAcceleration = yoTouchdownAcceleration;
      
      footContactPointInterpolator = new FootContactStateInterpolator(robotSide, contactState, controllerToolbox.getControlDT(), parentRegistry);

      SideDependentList<FootSwitchInterface> footSwitches = controllerToolbox.getFootSwitches();
      footSwitch = footSwitches.get(robotSide);
      soleFrame = contactableFoot.getSoleFrame();

      String namePrefix = footControlHelper.getRobotSide().getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(namePrefix + name);

      positionTrajectory = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "PositionTrajectory", worldFrame, registry);
      orientationTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(namePrefix + "OrientationTrajectory", worldFrame, registry);

      initialPosition = new YoFramePoint(namePrefix + "initialPosition", worldFrame, registry);
      initialOrientation = new YoFrameQuaternion(namePrefix + "initialOrientation", worldFrame, registry);
      initialAngularVelocity = new YoFrameVector(namePrefix + "initialAngularVelocity", worldFrame, registry);
      finalOrientation = new YoFrameQuaternion(namePrefix + "finalOrientation", worldFrame, registry);
      finalAngularVelocity = new YoFrameVector(namePrefix + "finalAngularVelocity", worldFrame, registry);

      usePointContact = new YoBoolean(namePrefix + "UsePointContact", registry);
      inContact = new YoBoolean(namePrefix + "InContact", registry);
      rampingContactNormals = new YoBoolean(namePrefix + "RampingContactNormals", registry);
      timeInContact = new YoDouble(namePrefix + "TimeInContact", registry);
      contactStartTime = new YoDouble(namePrefix + "ContactStartTime", registry);
      rampingContactNormalsStartTime = new YoDouble(namePrefix + "RampingContactNormalsStartTime", registry);
      desiredTouchdownDuration = new YoDouble(namePrefix + "DesiredTouchdownDuration", registry);
      percentageOfTouchdownDurationToInitiateRamping = new YoDouble(namePrefix + "percentOfDurationToStartRamping", registry);
      thresholdOfFootLoadingToAssumeInitialContact = new YoDouble(namePrefix + "thresholdOfFootLoadingToAssumeInitialContact", registry);
      contactNormalMaximumForce = new YoDouble(namePrefix + "ContactNormalMaximum", registry);

      percentageOfTouchdownDurationToInitiateRamping.set(0.15);
      thresholdOfFootLoadingToAssumeInitialContact.set(0.05);
      desiredTouchdownDuration.set(0.15);

      footBodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      groundContactFrame = new GroundContactReferenceFrame(namePrefix + "groundContactFrame", footBodyFixedFrame);
     
      feedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      feedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      feedbackControlCommand.setPrimaryBase(pelvis);
      feedbackControlCommand.setGains(swingFootControlGains);
      
      inContactFeedbackGains = new DefaultYoPIDSE3Gains("inContactFeedbackGains", swingFootControlGains.getOrientationGains(), swingFootControlGains.getPositionGains(), registry);
      inContactFeedbackGains.setOrientationProportionalGains(0.0);
      
      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
      ReferenceFrame toeFrame = createToeFrame(robotSide);
      controlFrame = walkingControllerParameters.controlToeDuringSwing() ? toeFrame : footFrame;
      
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(footBodyFixedFrame);
      feedbackControlCommand.setControlBaseFrame(footBodyFixedFrame);
      feedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);

      zeroAccelerationCommand.setWeight(SolverWeightLevels.HIGH);
      zeroAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      zeroAccelerationCommand.setPrimaryBase(pelvis);

      parentRegistry.addChild(registry);
   }
   
   private ReferenceFrame createToeFrame(RobotSide robotSide)
   {
      ContactableFoot contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      ReferenceFrame footFrame = controllerToolbox.getReferenceFrames().getFootFrame(robotSide);
      FramePoint2D toeContactPoint2d = new FramePoint2D();
      contactableFoot.getToeOffContactPoint(toeContactPoint2d);
      FramePoint3D toeContactPoint = new FramePoint3D();
      toeContactPoint.setIncludingFrame(toeContactPoint2d, 0.0);
      toeContactPoint.changeFrame(footFrame);

      tempRigidBodyTransform.setTranslation(toeContactPoint);
      return ReferenceFrame.constructFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() + "ToeFrame", footFrame, tempRigidBodyTransform);
   }


   public void setWeight(double weight)
   {
      feedbackControlCommand.setWeightForSolver(weight);
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      feedbackControlCommand.setWeightsForSolver(angular, linear);
   }

   public void setUsePointContact(boolean usePointContact)
   {
      this.usePointContact.set(usePointContact);
   }

   @Override
   public void doSpecificAction()
   {
      double timeInCurrentState = getTimeInCurrentState();
      double footLoadPercentage = footSwitch.computeFootLoadPercentage();

      if (hasFootMadeContactWithGround(footLoadPercentage))
      {
         if (!inContact.getBooleanValue())
         {
            inContact.set(true);
            contactState.setContactNormalVector(footControlHelper.getFullyConstrainedNormalContactVector());
            contactStartTime.set(timeInCurrentState);
            enableLineContact();
            configureControllerCoreCommandsAfterContact();
         }

         timeInContact.set(timeInCurrentState - contactStartTime.getDoubleValue());
         if (shouldRampContactNormals(timeInCurrentState))
         {
            footContactPointInterpolator.update();
            if (!rampingContactNormals.getBooleanValue())
            {
               rampingContactNormals.set(true);
               rampingContactNormalsStartTime.set(timeInCurrentState);
            }
            double timeInRamping = timeInCurrentState - rampingContactNormalsStartTime.getDoubleValue();
//            double rampingDuration = desiredTouchdownDuration.getDoubleValue() - rampingContactNormalsStartTime.getDoubleValue();
            double rampingDuration = 0.05;

            double percentage = (timeInRamping / rampingDuration) * 20;
            double forceMax = Math.exp(percentage);
            contactNormalMaximumForce.set(forceMax);
            rampMaximumContactNormals(forceMax);
         }
      }

      positionTrajectory.compute(timeInCurrentState);
      orientationTrajectory.compute(timeInCurrentState);

      positionTrajectory.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      orientationTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      feedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      feedbackControlCommand.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
   }

   private boolean shouldRampContactNormals(double timeInCurrentState)
   {
      return true;//timeInCurrentState > desiredTouchdownDuration.getDoubleValue() * percentageOfTouchdownDurationToInitiateRamping.getDoubleValue();
   }

   private void rampMaximumContactNormals(double maxNormalForce)
   {
      contactState.setFullyConstrained();
      for (int i = 2; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         contactState.setMaxContactPointNormalForce(yoContactPoint, maxNormalForce);
      }
   }

   private boolean hasFootMadeContactWithGround(double footLoadPercentage)
   {
      return footLoadPercentage > thresholdOfFootLoadingToAssumeInitialContact.getDoubleValue() || inContact.getBooleanValue();
   }

   private void configureControllerCoreCommandsAfterContact()
   {
      desiredLinearAcceleration.changeFrame(worldFrame);
      zeroAccelerationCommand.setLinearAcceleration(worldFrame, desiredLinearAcceleration);
      footAccelerationSelectionMatrix.setToLinearSelectionOnly();
      footAccelerationSelectionMatrix.setSelectionFrame(worldFrame);
      zeroAccelerationCommand.setSelectionMatrix(footAccelerationSelectionMatrix);

      feedbackControlCommand.setControlBaseFrame(groundContactFrame);
      feedbackControlSelectionMatrix.setSelectionFrame(groundContactFrame);
//      feedbackControlSelectionMatrix.selectAngularY(false);
      feedbackControlCommand.setSelectionMatrix(feedbackControlSelectionMatrix);
//      feedbackControlCommand.setGains(inContactFeedbackGains);
   }

   public void getDesireds(FrameOrientation desiredOrientationToPack, FrameVector3D desiredAngularVelocityToPack)
   {
      desiredOrientationToPack.setIncludingFrame(desiredOrientation);
      desiredAngularVelocityToPack.setIncludingFrame(desiredAngularVelocity);
   }

   private void enableLineContact()
   {
      updateLineContactBasedOnSensedCoP();
      MovingReferenceFrame footFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      contactLine.changeFrame(footFixedFrame);
      contactPointPosition.setToNaN(footFixedFrame);
      contactLine.getMidpoint(contactPointPosition);
      groundContactFrame.updateTranslation(contactPointPosition);
      
      footContactPointInterpolator.initialize(0.05, contactLine);

      ToeSlippingDetector toeSlippingDetector = footControlHelper.getToeSlippingDetector();
      if (toeSlippingDetector != null)
         toeSlippingDetector.initialize(contactPointPosition);
   }

   public void initialize(double desiredTouchdownDuration, FramePose initialFootPose, FrameVector3D initialFootAngularVelocity,
         FrameOrientation finalFootOrientation)
   {
      this.desiredTouchdownDuration.set(desiredTouchdownDuration);

      initialFootPose.changeFrame(worldFrame);
      initialFootAngularVelocity.changeFrame(worldFrame);
      finalFootOrientation.changeFrame(worldFrame);

      this.initialPosition.set(initialFootPose.getPosition());
      this.initialOrientation.set(initialFootPose.getOrientation());
      this.initialAngularVelocity.set(initialFootAngularVelocity);
      this.finalOrientation.set(finalFootOrientation);

      feedbackControlSelectionMatrix.resetSelection();
      feedbackControlCommand.setSelectionMatrix(feedbackControlSelectionMatrix);
      
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(footBodyFixedFrame);
      feedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
   }

   public void initialize(double desiredTouchdownDuration, FramePose initialFootPose, FrameVector3D initialFootLinearVelocity,
         FrameVector3D initialFootAngularVelocity)
   {
      this.desiredTouchdownDuration.set(desiredTouchdownDuration);
      initialFootPose.changeFrame(worldFrame);
      initialFootAngularVelocity.changeFrame(worldFrame);

      this.initialPosition.set(initialFootPose.getPosition());
      this.initialOrientation.set(initialFootPose.getOrientation());
      this.initialAngularVelocity.set(initialFootAngularVelocity);
      finalOrientation.set(initialFootPose.getOrientation().getYaw(), 0.0, 0.0);

      feedbackControlSelectionMatrix.resetSelection();
      feedbackControlCommand.setSelectionMatrix(feedbackControlSelectionMatrix);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      feedbackControlCommand.setGains(swingFootControlGains);

      inContact.set(false);
      rampingContactNormals.set(false);

      //      currentFootPose.setToZero(contactableFoot.getFrameAfterParentJoint());
      //      currentFootPose.changeFrame(worldFrame);
      //
      //      currentFootPose.getPosition(currentFootPosition);
      initialPosition.getFrameTupleIncludingFrame(currentFootPosition);
      positionTrajectory.initialize(0.0, currentFootPosition, desiredInitialTouchdownVelocity.getFrameTuple(),
            desiredInitialTouchdownAcceleration.getFrameTuple());

      orientationTrajectory.setInitialConditions(initialOrientation, initialAngularVelocity);
      orientationTrajectory.setFinalConditions(finalOrientation, finalAngularVelocity);
      orientationTrajectory.setNumberOfRevolutions(0);
      orientationTrajectory.setTrajectoryTime(desiredTouchdownDuration.getDoubleValue());
      orientationTrajectory.initialize();

   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      ToeSlippingDetector toeSlippingDetector = footControlHelper.getToeSlippingDetector();
      if (toeSlippingDetector != null)
         toeSlippingDetector.clear();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactState.setMaxContactPointNormalForce(contactPoints.get(i), Double.MAX_VALUE);
      }
      footContactPointInterpolator.resetContactState();
      
   }

   @Override
   public boolean isDone()
   {
      return getTimeInCurrentState() > desiredTouchdownDuration.getDoubleValue();
   }

   private final SearchDirectionFramePointComparator comparator = new SearchDirectionFramePointComparator();
   private final FramePoint3D endPointA = new FramePoint3D();
   private final FramePoint3D endPointB = new FramePoint3D();

   private void updateLineContactBasedOnSensedCoP()
   {
      footSwitch.computeAndPackCoP(sensedCoP);
      sensedCoP.changeFrame(soleFrame);
      comparator.setCoP(sensedCoP);
      ListSorter.sort(contactPoints, comparator);

      YoContactPoint yoContactPointA = contactPoints.get(0);
      yoContactPointA.setInContact(true);
      yoContactPointA.getPosition(endPointA);

      YoContactPoint yoContactPointB = contactPoints.get(1);
      yoContactPointB.setInContact(true);
      yoContactPointB.getPosition(endPointB);

      contactState.updateInContact();
//      contactState.notifyContactStateHasChanged();
      contactLine.setIncludingFrame(endPointA, endPointB);
   }

   private void updateLineContactBasedOnInitialFootOrientation()
   {
      //      contactLine.set(contactPoints2D.get(0), contactPoints2D.get(1));
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (inContact.getBooleanValue())
      {
         return zeroAccelerationCommand;
      }
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   private class GroundContactReferenceFrame extends MovingReferenceFrame
   {
      private static final long serialVersionUID = 7855074528733400796L;

      private final FrameVector3D originVector;

      public GroundContactReferenceFrame(String name, ReferenceFrame parentFrame)
      {
         super(name, parentFrame, new RigidBodyTransform(), false, true);
         originVector = new FrameVector3D(parentFrame);
      }

      @Override
      protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
      {

      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setIdentity();
         transformToParent.setTranslation(originVector.getVector());
      }

      public void updateTranslation(FrameTuple3D<?, ?> frameVector)
      {
         originVector.set(frameVector);
         this.update();
      }
   }

   private class SearchDirectionFramePointComparator implements Comparator<YoContactPoint>
   {
      private final FrameVector2D cop = new FrameVector2D();
      private final FrameVector2D cp1 = new FrameVector2D();
      private final FrameVector2D cp2 = new FrameVector2D();

      public void setCoP(FrameTuple2DReadOnly cop)
      {
         this.cop.setIncludingFrame(cop);
      }

      public int compare(YoContactPoint o1, YoContactPoint o2)
      {
         o1.getPosition2d(cp1);
         o2.getPosition2d(cp2);

         cp1.sub(cop);
         cp2.sub(cop);

         double cp1Distance = cp1.lengthSquared();
         double cp2Distance = cp2.lengthSquared();
         if (cp1Distance < cp2Distance)
            return -1;
         if (cp1Distance > cp2Distance)
            return 1;
         return 0;
      }
   }
}
