package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.contactPoints.ContactStateRhoRamping;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.referenceFrame.FrameLineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.tools.lists.ListSorter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * Attempts to soften the touchdown portion of swing. This state is only triggered if a touchdown duration is supplied with the footstep
 * This state is triggered after the foot comes in contact with the ground
 */
public class TouchDownState extends AbstractFootControlState
{
   
   private enum LineContactActivationMethod { SENSED_COP, FOOT_ORIENTATION };
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand zeroAccelerationCommand = new SpatialAccelerationCommand();

   private final SelectionMatrix6D footAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackControlSelectionMatrix = new SelectionMatrix6D();

   private final HermiteCurveBasedOrientationTrajectoryGenerator orientationTrajectory;
   private final ContactStateRhoRamping footContactRhoRamper;

   private final FrameLineSegment3D contactLine = new FrameLineSegment3D();

   private final YoEnum<LineContactActivationMethod> lineContactActivationMethod;
   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector3D initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector3D finalAngularVelocity;

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
   private final FramePose3D controlFramePose = new FramePose3D();

   private final YoContactPointDistanceComparator copDistanceComparator = new YoContactPointDistanceComparator();
   private final YoContactLowestPointDistanceComparator zHeightDistanceComparator = new YoContactLowestPointDistanceComparator();
   private final FramePoint3D endPointA = new FramePoint3D();
   private final FramePoint3D endPointB = new FramePoint3D();

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final PIDSE3GainsReadOnly gains;

   /**
    * Attempts to soften the touchdown portion of swing. This state is only triggered if a touchdown duration is supplied with the footstep
    * This state is triggered after the foot comes in contact with the ground
    * @param footControlHelper
    * @param swingFootControlGains
    * @param parentRegistry
    */
   public TouchDownState(FootControlHelper footControlHelper, PIDSE3GainsReadOnly swingFootControlGains, YoVariableRegistry parentRegistry)
   {
      super(footControlHelper);

      this.gains = swingFootControlGains;

      MomentumOptimizationSettings momentumOptimizationSettings = footControlHelper.getWalkingControllerParameters().getMomentumOptimizationSettings();

      SideDependentList<FootSwitchInterface> footSwitches = controllerToolbox.getFootSwitches();
      footSwitch = footSwitches.get(robotSide);
      soleFrame = contactableFoot.getSoleFrame();

      String namePrefix = footControlHelper.getRobotSide().getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(namePrefix + name);

      double rhoWeight = momentumOptimizationSettings.getRhoWeight();
      
      lineContactActivationMethod = new YoEnum<>("lineContactActivationMethod", registry, LineContactActivationMethod.class);
      lineContactActivationMethod.set(LineContactActivationMethod.FOOT_ORIENTATION);
      
      footContactRhoRamper = new ContactStateRhoRamping(robotSide, contactState, rhoWeight, registry);
      orientationTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(namePrefix + "OrientationTrajectory", worldFrame, registry);

      initialOrientation = new YoFrameQuaternion(namePrefix + "initialOrientation", worldFrame, registry);
      initialAngularVelocity = new YoFrameVector3D(namePrefix + "initialAngularVelocity", worldFrame, registry);
      finalOrientation = new YoFrameQuaternion(namePrefix + "finalOrientation", worldFrame, registry);
      finalAngularVelocity = new YoFrameVector3D(namePrefix + "finalAngularVelocity", worldFrame, registry);

      timeInContact = new YoDouble(namePrefix + "TimeInContact", registry);
      desiredTouchdownDuration = new YoDouble(namePrefix + "DesiredTouchdownDuration", registry);

      footBodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      groundContactFrame = new TranslationReferenceFrame(namePrefix + "groundContactFrame", footBodyFixedFrame);
      controlFrame = footBodyFixedFrame;

      feedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      feedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      feedbackControlCommand.setPrimaryBase(pelvis);

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

   @Override
   public void doSpecificAction(double timeInState)
   {
      timeInContact.set(timeInState);
      footContactRhoRamper.update(timeInState);

      orientationTrajectory.compute(timeInState);
      orientationTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      feedbackControlCommand.setInverseDynamics(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      feedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);
      feedbackControlCommand.setGains(gains);
   }
   
   public void setTouchdownDuration(double touchdownDuration)
   {
      desiredTouchdownDuration.set(touchdownDuration);
   }

   /**
    * This assumes flat ground! sets the final desired foot pose to assume flat ground and uses the foot's current desired yaw.
    * The pose initials should probably be the swing's final desireds.
    * @param desiredTouchdownDuration the time the orientation trajectory and contact point rho ramping take to complete
    * @param initialFootPose the current desired foot pose
    * @param initialFootLinearVelocity the current desired foot linear velocity
    * @param initialFootAngularVelocity the current desired foot angular velocity
    */
   public void initialize(FramePose3D initialFootPose, FrameVector3D initialFootLinearVelocity,
         FrameVector3D initialFootAngularVelocity)
   {
      initialFootPose.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.setYawPitchRoll(initialFootPose.getYaw(), 0.0, 0.0);
      initialize(initialFootPose, initialFootLinearVelocity, initialFootAngularVelocity, desiredOrientation);
   }

   /**
    * initialize the touchdown state. The pose initials should probably be the swing's final desireds.
    * @param desiredTouchdownDuration  the time the orientation trajectory and contact point rho ramping take to complete
    * @param initialFootPose the current desired foot pose
    * @param initialFootLinearVelocity  the current desired foot linear velocity
    * @param initialFootAngularVelocity the current desired foot angular velocity
    * @param finalFootOrientation the final orientation trajectory desired, this should be the ground orientation. 
    */
   public void initialize(FramePose3D initialFootPose, FrameVector3D initialFootLinearVelocity,
         FrameVector3D initialFootAngularVelocity, FrameQuaternion finalFootOrientation)
   {
      //We're only using the orientation here. If you want to switch to holding X and Y, be careful, as the swing tracks position in a weird frame, 
      //and once in contact, the desired position would need to be consistent with current desireds, but transformed to somewhere on the contact line
      initialFootPose.changeFrame(worldFrame);
      initialFootAngularVelocity.changeFrame(worldFrame);
      finalFootOrientation.changeFrame(worldFrame);

      this.initialOrientation.set(initialFootPose.getOrientation());
      this.initialAngularVelocity.set(initialFootAngularVelocity);
      this.finalOrientation.set(finalFootOrientation);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();
      
      enableLineContactAndSetTheGroundContactFrame(contactPointPosition, groundContactFrame);
      footContactRhoRamper.initialize(desiredTouchdownDuration.getDoubleValue());

      zeroAccelerationCommand.setSpatialAccelerationToZero(groundContactFrame);
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);

      orientationTrajectory.setInitialConditions(initialOrientation, initialAngularVelocity);
      orientationTrajectory.setFinalConditions(finalOrientation, finalAngularVelocity);
      orientationTrajectory.setNumberOfRevolutions(0);
      orientationTrajectory.setTrajectoryTime(desiredTouchdownDuration.getDoubleValue());
      orientationTrajectory.initialize();
   }

   @Override
   public void onExit()
   {
      super.onExit();
      footContactRhoRamper.resetContactState();
   }

   /**
    * extracts a line contact
    * sets the contactPointPosition to the midpoint of the contact line
    * initialized the rhoRamping and sets all contact points in contact
    * @param groundContactFrameToUpdate 
    * @param contactPointPositionToPack 
    */
   private void enableLineContactAndSetTheGroundContactFrame(FramePoint3D contactPointPositionToPack, TranslationReferenceFrame groundContactFrameToUpdate)
   {
      //choose a way to sort the contacts
      Comparator<YoContactPoint> contactPointComparator = zHeightDistanceComparator;
      if(lineContactActivationMethod.getEnumValue() == LineContactActivationMethod.SENSED_COP)
      {
         contactPointComparator = getComparatorBasedOnCoPPosition();
      }
      
      updateUsingLineContact(contactState, contactPoints, contactLine, contactPointComparator);
      MovingReferenceFrame footFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      contactLine.changeFrame(footFixedFrame);
      contactPointPositionToPack.setToNaN(footFixedFrame);
      contactLine.midpoint(contactPointPositionToPack);
      groundContactFrameToUpdate.updateTranslation(contactPointPositionToPack);
   }
   
   /**
    * updates and returns the {@link copDistanceComparator} with sensed Center of Pressure.
    * @return
    */
   private Comparator<YoContactPoint> getComparatorBasedOnCoPPosition()
   {
      footSwitch.computeAndPackCoP(sensedCoP);
      sensedCoP.changeFrame(soleFrame);
      copDistanceComparator.setCoP(sensedCoP);
      return copDistanceComparator;
   }
   
   /**
    * extracts a line contact by sorting the contact points based on the {@link contactPointComparator} and uses the first 2 in the list
    * @param contactState used to update the in contact state after enabled the contact points
    * @param contactPoints used to find the closest contact points from the sensed CoP
    * @param contactLineToPack the line contact found 
    */
   private void updateUsingLineContact(YoPlaneContactState contactState, List<YoContactPoint> contactPoints, FrameLineSegment3D contactLineToPack, Comparator<YoContactPoint> contactPointComparator)
   {
      ListSorter.sort(contactPoints, contactPointComparator);

      YoContactPoint yoContactPointA = contactPoints.get(0);
      yoContactPointA.setInContact(true);
      yoContactPointA.getPosition(endPointA);

      YoContactPoint yoContactPointB = contactPoints.get(1);
      yoContactPointB.setInContact(true);
      yoContactPointB.getPosition(endPointB);

      contactState.updateInContact();
      contactLineToPack.setIncludingFrame(endPointA, endPointB);
      contactState.setContactNormalVector(footControlHelper.getFullyConstrainedNormalContactVector());
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight = angularWeight;
      this.linearWeight = linearWeight;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return zeroAccelerationCommand;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > desiredTouchdownDuration.getDoubleValue();
   }

   /**
    * Compares YoContactPoints against the distance from a point (in this case the Center of Pressure)
    */
   private class YoContactPointDistanceComparator implements Comparator<YoContactPoint>
   {
      private final FrameVector2D cop = new FrameVector2D();
      private final FrameVector2D cp1 = new FrameVector2D();
      private final FrameVector2D cp2 = new FrameVector2D();

      public void setCoP(FrameTuple2DReadOnly cop)
      {
         this.cop.setIncludingFrame(cop);
      }

      @Override
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

   /**
    * Compares YoContactPoints Z heights, This assumes the foot is hitting flat ground. 
    */
   private class YoContactLowestPointDistanceComparator implements Comparator<YoContactPoint>
   {
      private final FramePoint3D cp1 = new FramePoint3D();
      private final FramePoint3D cp2 = new FramePoint3D();
      
      @Override
      public int compare(YoContactPoint o1, YoContactPoint o2)
      {
         o1.getPosition(cp1);
         o2.getPosition(cp2);
         
         cp1.changeFrame(worldFrame);
         cp2.changeFrame(worldFrame);
         
         if (cp1.getZ() < cp2.getZ())
            return -1;
         if (cp1.getZ() > cp2.getZ())
            return 1;
         return 0;
      }
   }
}
