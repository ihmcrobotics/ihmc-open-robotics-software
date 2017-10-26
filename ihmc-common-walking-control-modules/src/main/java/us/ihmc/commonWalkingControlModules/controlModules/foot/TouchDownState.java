package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.geometry.FrameLineSegment;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.tools.lists.ListSorter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TouchDownState extends AbstractFootControlState
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand zeroAccelerationCommand = new SpatialAccelerationCommand();

   private final SelectionMatrix6D footAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackControlSelectionMatrix = new SelectionMatrix6D();

   private final HermiteCurveBasedOrientationTrajectoryGenerator orientationTrajectory;
   private final FootContactStateInterpolator footContactPointInterpolator;

   private final FrameLineSegment contactLine = new FrameLineSegment();

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector finalAngularVelocity;

   private final FramePoint3D contactPointPosition = new FramePoint3D();

   private final YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
   private final List<YoContactPoint> contactPoints = new ArrayList<>(contactState.getContactPoints());

   private final YoDouble timeInContact;
   private final YoDouble desiredTouchdownDuration;

   private final TranslationReferenceFrame groundContactFrame;
   private final FootSwitchInterface footSwitch;
   private final FramePoint2D sensedCoP = new FramePoint2D();
   private final ReferenceFrame soleFrame;

   private final ReferenceFrame controlFrame;
   private final MovingReferenceFrame footBodyFixedFrame;
   private final FramePose controlFramePose = new FramePose();

   public TouchDownState(FootControlHelper footControlHelper, YoPIDSE3Gains swingFootControlGains, YoVariableRegistry parentRegistry)
   {
      super(ConstraintType.TOUCHDOWN, footControlHelper);

      footContactPointInterpolator = new FootContactStateInterpolator(robotSide, contactState, controllerToolbox.getControlDT(), parentRegistry);

      SideDependentList<FootSwitchInterface> footSwitches = controllerToolbox.getFootSwitches();
      footSwitch = footSwitches.get(robotSide);
      soleFrame = contactableFoot.getSoleFrame();

      String namePrefix = footControlHelper.getRobotSide().getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(namePrefix + name);

      orientationTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(namePrefix + "OrientationTrajectory", worldFrame, registry);

      initialOrientation = new YoFrameQuaternion(namePrefix + "initialOrientation", worldFrame, registry);
      initialAngularVelocity = new YoFrameVector(namePrefix + "initialAngularVelocity", worldFrame, registry);
      finalOrientation = new YoFrameQuaternion(namePrefix + "finalOrientation", worldFrame, registry);
      finalAngularVelocity = new YoFrameVector(namePrefix + "finalAngularVelocity", worldFrame, registry);

      timeInContact = new YoDouble(namePrefix + "TimeInContact", registry);
      desiredTouchdownDuration = new YoDouble(namePrefix + "DesiredTouchdownDuration", registry);

      footBodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      groundContactFrame = new TranslationReferenceFrame(namePrefix + "groundContactFrame", footBodyFixedFrame);
      controlFrame = footBodyFixedFrame;

      feedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      feedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      feedbackControlCommand.setPrimaryBase(pelvis);
      feedbackControlCommand.setGains(swingFootControlGains);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(footBodyFixedFrame);
      feedbackControlCommand.setControlBaseFrame(footBodyFixedFrame);
      feedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      feedbackControlSelectionMatrix.setToAngularSelectionOnly();
      feedbackControlCommand.setSelectionMatrix(feedbackControlSelectionMatrix);

      zeroAccelerationCommand.setWeight(SolverWeightLevels.HIGH);
      zeroAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      zeroAccelerationCommand.setPrimaryBase(pelvis);

      footAccelerationSelectionMatrix.setToLinearSelectionOnly();
      footAccelerationSelectionMatrix.setSelectionFrame(worldFrame);
      zeroAccelerationCommand.setSelectionMatrix(footAccelerationSelectionMatrix);

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      feedbackControlCommand.setWeightForSolver(weight);
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      feedbackControlCommand.setWeightsForSolver(angular, linear);
   }

   @Override
   public void doSpecificAction()
   {
      double timeInCurrentState = getTimeInCurrentState();

      timeInContact.set(timeInCurrentState);
      footContactPointInterpolator.update();

      orientationTrajectory.compute(timeInCurrentState);
      orientationTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      feedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   private void enableLineContact()
   {
      updateContactStateForLineContactBasedOnSensedCoP();
      MovingReferenceFrame footFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      contactLine.changeFrame(footFixedFrame);
      contactPointPosition.setToNaN(footFixedFrame);
      contactLine.getMidpoint(contactPointPosition);
      groundContactFrame.updateTranslation(contactPointPosition);

      contactState.setContactNormalVector(footControlHelper.getFullyConstrainedNormalContactVector());
      footContactPointInterpolator.initialize(desiredTouchdownDuration.getDoubleValue(), contactLine);
   }

   /**
    * This assumes flat ground, sets the final desired foot pose to assume flat ground and uses the foots current desired yaw
    * @param desiredTouchdownDuration
    * @param initialFootPose
    * @param initialFootLinearVelocity
    * @param initialFootAngularVelocity
    */
   public void initialize(double desiredTouchdownDuration, FramePose initialFootPose, FrameVector3D initialFootLinearVelocity,
         FrameVector3D initialFootAngularVelocity)
   {
      initialFootPose.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), 0.0, 0.0);
      initialize(desiredTouchdownDuration, initialFootPose, initialFootLinearVelocity, initialFootAngularVelocity, desiredOrientation);
   }

   public void initialize(double desiredTouchdownDuration, FramePose initialFootPose, FrameVector3D initialFootLinearVelocity,
         FrameVector3D initialFootAngularVelocity, FrameOrientation finalFootOrientation)
   {
      this.desiredTouchdownDuration.set(desiredTouchdownDuration);

      initialFootPose.changeFrame(worldFrame);
      initialFootAngularVelocity.changeFrame(worldFrame);
      finalFootOrientation.changeFrame(worldFrame);

      this.initialOrientation.set(initialFootPose.getOrientation());
      this.initialAngularVelocity.set(initialFootAngularVelocity);
      this.finalOrientation.set(finalFootOrientation);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      enableLineContact();

      zeroAccelerationCommand.setSpatialAccelerationToZero(groundContactFrame);
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);

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

   private void updateContactStateForLineContactBasedOnSensedCoP()
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
      return zeroAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
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
