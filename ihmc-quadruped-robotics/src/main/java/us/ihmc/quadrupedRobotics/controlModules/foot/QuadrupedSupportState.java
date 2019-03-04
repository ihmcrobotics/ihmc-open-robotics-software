package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final YoPlaneContactState contactState;
   private final ReferenceFrame soleFrame;

   private final FootSwitchInterface footSwitch;

   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
   private boolean footIsVerifiedAsLoaded = false;

   private final DoubleParameter minimumTimeInSupportState;

   private final YoFramePoint3D groundPlanePosition;
   private final YoFramePoint3D upcomingGroundPlanePosition;

   private final SpatialAcceleration footAcceleration = new SpatialAcceleration();

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final RigidBodyBasics rootBody;

   private final Vector3DReadOnly linearWeight;

   private final YoBoolean footBarelyLoaded;
   private final DoubleParameter footLoadThreshold;

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.groundPlanePosition = controllerToolbox.getGroundPlanePositions().get(robotQuadrant);
      this.upcomingGroundPlanePosition = controllerToolbox.getUpcomingGroundPlanePositions().get(robotQuadrant);
      this.contactState = controllerToolbox.getFootContactState(robotQuadrant);
      this.soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);

      rootBody = controllerToolbox.getFullRobotModel().getElevator();

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactState.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(controllerToolbox.getFullRobotModel().getBody());
      spatialAccelerationCommand.setSelectionMatrixForLinearControl();

      minimumTimeInSupportState = new DoubleParameter(robotQuadrant.getShortName() + "TimeInSupportState", registry, 0.05);
      linearWeight = new ParameterVector3D(robotQuadrant.getShortName() + "_supportFootWeight", new Vector3D(10.0, 10.0, 10.0), registry);

      footBarelyLoaded = new YoBoolean(robotQuadrant.getShortName() + "_BarelyLoaded", registry);
      footLoadThreshold = new DoubleParameter(robotQuadrant.getShortName() + "_FootLoadThreshold", registry, 0.15);

      footSwitch = controllerToolbox.getRuntimeEnvironment().getFootSwitches().get(robotQuadrant);
   }

   @Override
   public void onEntry()
   {
      contactState.setFullyConstrained();
      contactState.setContactNormalVector(footNormalContactVector);

      if (waypointCallback != null)
         waypointCallback.isDoneMoving(robotQuadrant, true);

      footIsVerifiedAsLoaded = false;


      footBarelyLoaded.set(false);
   }


   private final FramePoint3D tempPoint = new FramePoint3D();
   @Override
   public void doAction(double timeInState)
   {
      ReferenceFrame bodyFixedFrame = contactState.getRigidBody().getBodyFixedFrame();
      footAcceleration.setToZero(bodyFixedFrame, rootBody.getBodyFixedFrame(), soleFrame);
      footAcceleration.setBodyFrame(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(soleFrame, footAcceleration);
      spatialAccelerationCommand.setLinearWeights(linearWeight);

      if (footSwitch.hasFootHitGround())
      {
         if (!footIsVerifiedAsLoaded && timeInState > minimumTimeInSupportState.getValue())
         {
            footIsVerifiedAsLoaded = true;

            tempPoint.setToZero(soleFrame);
            groundPlanePosition.setMatchingFrame(tempPoint);
            upcomingGroundPlanePosition.setMatchingFrame(tempPoint);
         }
      }

      footBarelyLoaded.set(footSwitch.computeFootLoadPercentage() < footLoadThreshold.getValue());
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
      footIsVerifiedAsLoaded = false;
      footBarelyLoaded.set(false);
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}
