package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ExploreFootPolygonState extends AbstractFootControlState
{
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final FrameVector fullyConstrainedNormalContactVector;
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   
   private final FramePoint2d cop = new FramePoint2d();
   private final FramePoint2d desiredCoP = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;
   
   private final FootSwitchInterface footSwitch;
   
   private final DoubleYoVariable lastShrunkTime, spiralAngle;
   
   public ExploreFootPolygonState(FootControlHelper footControlHelper, OrientationPIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.EXPLORE_POLYGON, footControlHelper, registry);

      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setSelectionMatrixToIdentity();

      centerOfPressureCommand.setContactingRigidBody(contactableFoot.getRigidBody());
      centerOfPressureCommand.setWeight(new Vector2d(2000.0, 2000.0));

      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandList.addCommand(centerOfPressureCommand);

      lastShrunkTime = new DoubleYoVariable(contactableFoot.getName() + "LastShrunkTime", registry);
      spiralAngle = new DoubleYoVariable(contactableFoot.getName() + "SpiralAngle", registry);

      orientationFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      desiredOrientation.setToZero();
      desiredAngularVelocity.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);
      orientationFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      //TODO: Gains for damping, but not necessarily position control
      orientationFeedbackControlCommand.setGains(gains);
   }

   public void setWeight(double weight)
   {
      spatialAccelerationCommand.setWeight(weight);
   }

   public void setWeights(Vector3d angular, Vector3d linear)
   {
      spatialAccelerationCommand.setWeights(angular, linear);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableFoot, fullyConstrainedNormalContactVector);
      lastShrunkTime.set(0.0);
      spiralAngle.set(0.0);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
   }

   private final FramePoint2d shrunkPolygonCentroid = new FramePoint2d();
   private final FramePoint2d desiredCenterOfPressure = new FramePoint2d();
   @Override
   public void doSpecificAction()
   {
      footSwitch.computeAndPackCoP(cop);
      momentumBasedController.getDesiredCenterOfPressure(contactableFoot, desiredCoP);
      partialFootholdControlModule.compute(desiredCoP, cop);
      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
      boolean contactStateHasChanged = partialFootholdControlModule.applyShrunkPolygon(contactState);
      if (contactStateHasChanged)
      {
         contactState.notifyContactStateHasChanged();
         lastShrunkTime.set(getTimeInCurrentState());
         spiralAngle.add(Math.PI/2.0);
      }

      footAcceleration.setToZero(contactableFoot.getFrameAfterParentJoint(), rootBody.getBodyFixedFrame(), contactableFoot.getFrameAfterParentJoint());

      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(footAcceleration);

      // Foot exploration through CoP shifting...
      double timeInCurrentState = getTimeInCurrentState();
      double freq = 0.6;
      double rampOutDuration = 0.3;
      double settleDuration = 0.1;

      double percentRampOut = (timeInCurrentState - lastShrunkTime.getDoubleValue() - settleDuration) / rampOutDuration;
      rampOutDuration = MathTools.clipToMinMax(rampOutDuration, 0.0, 1.0);

      boolean doSpiral = timeInCurrentState - lastShrunkTime.getDoubleValue() - settleDuration > rampOutDuration;

      if (doSpiral)
      {
         double dt = 0.004; //Hack! Subtract from previous tick time or construct with dt!
         spiralAngle.add(2.0 * Math.PI * freq * dt);
      }

      ReferenceFrame soleFrame = footControlHelper.getContactableFoot().getSoleFrame();
      partialFootholdControlModule.getShrunkPolygonCentroid(shrunkPolygonCentroid);
      shrunkPolygonCentroid.changeFrame(soleFrame);

      desiredCenterOfPressure.setIncludingFrame(soleFrame, shrunkPolygonCentroid.getX() + 0.10 * percentRampOut * Math.cos(spiralAngle.getDoubleValue()), shrunkPolygonCentroid.getY() + 0.05 * percentRampOut * Math.sin(spiralAngle.getDoubleValue()));
      
      partialFootholdControlModule.projectOntoShrunkenPolygon(desiredCenterOfPressure);
      desiredCenterOfPressure.scale(0.9);
      centerOfPressureCommand.setDesiredCoP(desiredCenterOfPressure.getPoint());
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandList.addCommand(centerOfPressureCommand);
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
//      return orientationFeedbackControlCommand;
   }

}

