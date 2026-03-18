package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.LoadBearingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPositionControlHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicLoadBearingPreContactState implements DynamicLoadBearingState
{
   private final MovingReferenceFrame controlFrame;
   private final RigidBodyPositionControlHelper positionControlHelper;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FramePoint3D waypointPosition = new FramePoint3D();
   private final FrameVector3D terminalVelocity = new FrameVector3D();

   private final YoDouble trajectoryDuration;
   private final YoDouble distanceToPlane;

   private final Plane3DReadOnly bracingPlane;
   private final YoFramePoint3D yoBracingPoint;
   private final YoFrameVector3D yoBracingNormal;
   private final YoFramePose3D yoControlFrame;
   private final FramePose3D controlFramePose = new FramePose3D();

   private final FrameVector3D previousHandVelocity = new FrameVector3D();
   private final FrameVector3D handVelocity = new FrameVector3D();
   private final YoDouble handSpeed;
   private final GlitchFilteredYoBoolean hasHandTouchedDown;
   private final LoadBearingParameters loadBearingParameters;

   private final FramePoint3D currentPosition = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint2D tempPoint2d = new FramePoint2D();

   private final EuclideanTrajectoryControllerCommand trajectoryCommand = new EuclideanTrajectoryControllerCommand();

   private final RigidBodyTransform transformFromWorld = new RigidBodyTransform();
   private final ConvexPolygon2D regionPolygon = new ConvexPolygon2D();

   public DynamicLoadBearingPreContactState(RigidBodyBasics bodyToControl,
                                            RigidBodyPositionControlHelper positionControlHelper,
                                            ReferenceFrame controlFrame,
                                            LoadBearingParameters loadBearingParameters,
                                            Plane3DReadOnly bracingPlane,
                                            YoRegistry registry)
   {
      this.positionControlHelper = positionControlHelper;
      this.loadBearingParameters = loadBearingParameters;

      trajectoryDuration = new YoDouble("trajectoryDuration", registry);
      handSpeed = new YoDouble("handSpeed", registry);
      hasHandTouchedDown = new GlitchFilteredYoBoolean("hasHandTouchedDown", registry, 2);

      distanceToPlane = new YoDouble("distanceToPlane", registry);

      trajectoryCommand.setExecutionMode(ExecutionMode.OVERRIDE);
      trajectoryCommand.setUseCustomControlFrame(true);
      trajectoryCommand.setTrajectoryFrame(ReferenceFrame.getWorldFrame());
      controlFrame.getTransformToDesiredFrame(trajectoryCommand.getControlFramePose(), bodyToControl.getBodyFixedFrame());

      yoBracingPoint = new YoFramePoint3D(bodyToControl.getName() + "BracingPoint", ReferenceFrame.getWorldFrame(), registry);
      yoBracingNormal = new YoFrameVector3D(bodyToControl.getName() + "BracingNormal", ReferenceFrame.getWorldFrame(), registry);
      yoControlFrame = new YoFramePose3D(bodyToControl.getName() + "ControlFrame", ReferenceFrame.getWorldFrame(), registry);

      controlFramePose.setToZero(controlFrame);
      this.controlFrame = (MovingReferenceFrame) controlFrame;

      this.bracingPlane = bracingPlane;
   }

   public void setBracingData(HandContactCommand command)
   {
      this.desiredPosition.set(command.getBracingPoint());
      this.trajectoryDuration.set(command.getTrajectoryDuration());

      yoBracingPoint.set(command.getBracingPoint());
      yoBracingNormal.set(command.getBracingNormal());

      transformFromWorld.set(command.getRegionTransformFromWorld());
      regionPolygon.set(command.getConvexPolygon());
   }

   @Override
   public void onEntry()
   {
      currentPosition.setToZero(controlFrame);
      currentPosition.changeFrame(ReferenceFrame.getWorldFrame());
//      tempVector.sub(currentPosition, desiredPosition);
//      tempVector.normalize();
//      terminalHandSpeed.set(tempVector.dot(bracingPlane.getNormal()) * MAX_TERMINAL_HAND_SPEED);
//      if (terminalHandSpeed.getValue() < MIN_TERMINAL_HAND_SPEED)
//         terminalHandSpeed.set(MIN_TERMINAL_HAND_SPEED);

      terminalVelocity.setAndScale(-loadBearingParameters.getTerminalHandSpeed(), bracingPlane.getNormal());

      handVelocity.setIncludingFrame(controlFrame.getTwistOfFrame().getLinearPart());
      handVelocity.changeFrame(ReferenceFrame.getWorldFrame());

//      double waypointOffset = 0.1;
//      waypointPosition.set(yoBracingNormal);
//      waypointPosition.scale(waypointOffset);

      double lookAheadDt = 0.2;
      waypointPosition.setIncludingFrame(desiredPosition);
      tempVector.setAndScale(lookAheadDt, terminalVelocity);
      waypointPosition.add(tempVector);

      trajectoryCommand.getTrajectoryPointList().clear();
      trajectoryCommand.addTrajectoryPoint(0.0, currentPosition, handVelocity);
      trajectoryCommand.addTrajectoryPoint(trajectoryDuration.getValue(), desiredPosition, terminalVelocity);
      trajectoryCommand.addTrajectoryPoint(trajectoryDuration.getValue() + lookAheadDt, waypointPosition, terminalVelocity);
      positionControlHelper.handleTrajectoryCommand(trajectoryCommand, null);
      positionControlHelper.doAction(0.0);

      positionControlHelper.setWeights(loadBearingParameters.getPreContactPositionWeights());
      positionControlHelper.setGains(loadBearingParameters.getPreContactFeedbackGains().getPositionGains());

      hasHandTouchedDown.set(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      positionControlHelper.doAction(timeInState);
      yoControlFrame.setMatchingFrame(controlFramePose);
   }

   @Override
   public void onExit(double timeInState)
   {
      yoBracingPoint.setToNaN();
      yoBracingNormal.setToNaN();
      yoControlFrame.setToNaN();
   }

   @Override
   public boolean isDone(double timeInState)
   {
//      double epsilonCloseToWall = 0.012; // sim
      double epsilonCloseToWall = 0.04; // real robot
      distanceToPlane.set(bracingPlane.distance(positionControlHelper.getYoCurrentPosition()));
      boolean isCloseToWall = distanceToPlane.getValue() < epsilonCloseToWall;

      tempVector.setIncludingFrame(controlFrame.getTwistOfFrame().getLinearPart());
      tempVector.changeFrame(ReferenceFrame.getWorldFrame());
      handSpeed.set(Math.abs(tempVector.dot(yoBracingNormal)));
      boolean hasLowHandSpeed = handSpeed.getValue() < 0.1;

//      tempPoint.set(positionControlHelper.getYoCurrentPosition());
//      transformFromWorld.transform(tempPoint);
//      tempPoint2d.set(tempPoint);

//      double distanceFromRegion = regionPolygon.signedDistance(tempPoint2d);
//      boolean isInsideRegion = distanceFromRegion < 0.01;

      hasHandTouchedDown.update(isCloseToWall && hasLowHandSpeed && timeInState > 0.5 * trajectoryDuration.getValue());
      return hasHandTouchedDown.getValue();
   }

   public boolean isStuck(double timeInState)
   {
      return timeInState > trajectoryDuration.getValue() + 0.5;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return positionControlHelper.getFeedbackControlCommand();
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return null;
   }

   @Override
   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      return null;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(yoBracingPoint.getNamePrefix(),
                                                                    yoBracingPoint,
                                                                    0.01,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(yoBracingNormal.getNamePrefix(),
                                                                    yoBracingPoint,
                                                                    yoBracingNormal,
                                                                    0.25,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D("controlFrame", yoControlFrame, 0.15));
      return group;
   }
}
