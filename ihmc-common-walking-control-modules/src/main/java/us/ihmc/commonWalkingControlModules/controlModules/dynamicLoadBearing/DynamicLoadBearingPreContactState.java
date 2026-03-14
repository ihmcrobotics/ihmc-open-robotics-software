package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPositionControlHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicLoadBearingPreContactState implements DynamicLoadBearingState
{
   private static final double MIN_TERMINAL_HAND_SPEED = 0.8;
   private static final double MAX_TERMINAL_HAND_SPEED = 1.6;

   private final MovingReferenceFrame controlFrame;
   private final RigidBodyPositionControlHelper positionControlHelper;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D terminalVelocity = new FrameVector3D();
   private final Plane3D bracingPlane = new Plane3D();

   private final YoDouble trajectoryDuration;

   private final DefaultYoPIDSE3Gains bracingFeedbackGains;
   private final YoFrameVector3D bracingPositionWeights;
   private final YoFrameVector3D bracingOrientationWeights;

   private PID3DGainsReadOnly defaultPositionGains = null;
   private Vector3DReadOnly defaultPositionWeights = null;
   private final YoDouble distanceToPlane;

   private final YoFramePoint3D yoBracingPoint;
   private final YoFrameVector3D yoBracingNormal;
   private final YoFramePose3D yoControlFrame;
   private final FramePose3D controlFramePose = new FramePose3D();

   private final FrameVector3D previousHandVelocity = new FrameVector3D();
   private final FrameVector3D handVelocity = new FrameVector3D();
   private final YoDouble handSpeed;
   private final YoDouble terminalHandSpeed;
   private final GlitchFilteredYoBoolean hasHandTouchedDown;

   private final FramePoint3D currentPosition = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private final EuclideanTrajectoryControllerCommand trajectoryCommand = new EuclideanTrajectoryControllerCommand();

   public DynamicLoadBearingPreContactState(RigidBodyBasics bodyToControl,
                                            RigidBodyPositionControlHelper positionControlHelper,
                                            ReferenceFrame controlFrame,
                                            YoRegistry registry)
   {
      this.positionControlHelper = positionControlHelper;

      trajectoryDuration = new YoDouble("trajectoryDuration", registry);
      handSpeed = new YoDouble("handSpeed", registry);
      terminalHandSpeed = new YoDouble("terminalHandSpeed", registry);
      hasHandTouchedDown = new GlitchFilteredYoBoolean("", registry, 4);

      bracingPositionWeights = new YoFrameVector3D("bracingPositionWeights", ReferenceFrame.getWorldFrame(), registry);
      bracingOrientationWeights = new YoFrameVector3D("bracingOrientationWeights", ReferenceFrame.getWorldFrame(), registry);

      bracingPositionWeights.set(10.0, 10.0, 10.0);
      bracingOrientationWeights.set(0.0, 0.0, 0.0);

      bracingFeedbackGains = new DefaultYoPIDSE3Gains("PosDynamicLoadBearing", GainCoupling.XYZ, false, registry);
      configureGains();

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
   }

   public void setBracingPoint(Point3DReadOnly bracingPoint, Vector3DReadOnly bracingNormal, double trajectoryDuration)
   {
      this.desiredPosition.set(bracingPoint);
      this.bracingPlane.set(bracingPoint, bracingNormal);
      this.trajectoryDuration.set(trajectoryDuration);

      yoBracingPoint.set(bracingPoint);
      yoBracingNormal.set(bracingNormal);
   }

   @Override
   public void onEntry()
   {
      currentPosition.setToZero(controlFrame);
      currentPosition.changeFrame(ReferenceFrame.getWorldFrame());
      tempVector.sub(currentPosition, desiredPosition);
      tempVector.normalize();
      terminalHandSpeed.set(tempVector.dot(bracingPlane.getNormal()) * MAX_TERMINAL_HAND_SPEED);
      if (terminalHandSpeed.getValue() < MIN_TERMINAL_HAND_SPEED)
         terminalHandSpeed.set(MIN_TERMINAL_HAND_SPEED);

      terminalVelocity.setAndScale(-terminalHandSpeed.getDoubleValue(), bracingPlane.getNormal());

      handVelocity.setIncludingFrame(controlFrame.getTwistOfFrame().getLinearPart());
      handVelocity.changeFrame(ReferenceFrame.getWorldFrame());

      trajectoryCommand.getTrajectoryPointList().clear();
      trajectoryCommand.addTrajectoryPoint(0.0, currentPosition, handVelocity);
      trajectoryCommand.addTrajectoryPoint(trajectoryDuration.getValue(), desiredPosition, terminalVelocity);
      positionControlHelper.handleTrajectoryCommand(trajectoryCommand, null);
      positionControlHelper.doAction(0.0);

      defaultPositionGains = positionControlHelper.getGains();
      defaultPositionWeights = positionControlHelper.getDefaultWeight();
      positionControlHelper.setWeights(bracingPositionWeights);

      positionControlHelper.setGains(bracingFeedbackGains.getPositionGains());
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
      positionControlHelper.setGains(defaultPositionGains);
      positionControlHelper.setWeights(defaultPositionWeights);

      yoBracingPoint.setToNaN();
      yoBracingNormal.setToNaN();
      yoControlFrame.setToNaN();
   }

   @Override
   public boolean isDone(double timeInState)
   {
//      double epsilonCloseToWall = 0.012; // sim
      double epsilonCloseToWall = 0.02; // real robot
      distanceToPlane.set(bracingPlane.distance(positionControlHelper.getYoCurrentPosition()));
      boolean isCloseToWall = distanceToPlane.getValue() < epsilonCloseToWall;

      handSpeed.set(controlFrame.getTwistOfFrame().getLinearPart().norm());
      boolean hasLowHandSpeed = handSpeed.getValue() < 0.2;

      hasHandTouchedDown.update(isCloseToWall && hasLowHandSpeed);
      return hasHandTouchedDown.getValue();
   }

   public boolean isStuck(double timeInState)
   {
      return timeInState > trajectoryDuration.getValue() + 0.5;
   }

   private void configureGains()
   {
      double kpPosition = 100.0;
      double zetaPosition = 0.7;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      bracingFeedbackGains.setPositionProportionalGains(kpPosition);
      bracingFeedbackGains.setPositionDerivativeGains(GainCalculator.computeDerivativeGain(kpPosition, zetaPosition));
      bracingFeedbackGains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);

      double kpOrientation = 0.0;
      double kdOrientation = 12.0;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;
      bracingFeedbackGains.setOrientationProportionalGains(kpOrientation);
      bracingFeedbackGains.setOrientationDerivativeGains(kdOrientation);
      bracingFeedbackGains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);
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
