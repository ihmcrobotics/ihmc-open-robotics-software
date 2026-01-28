package us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPositionControlHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ReactiveBracingPreContactState implements ReactiveBracingState
{
   private static final double terminalHandSpeed = 1.2;

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

   private final EuclideanTrajectoryControllerCommand trajectoryCommand = new EuclideanTrajectoryControllerCommand();

   public ReactiveBracingPreContactState(RigidBodyBasics bodyToControl,
                                         RigidBodyPositionControlHelper positionControlHelper,
                                         ReferenceFrame controlFrame,
                                         YoRegistry registry)
   {
      this.positionControlHelper = positionControlHelper;

      trajectoryDuration = new YoDouble("trajectoryDuration", registry);

      bracingPositionWeights = new YoFrameVector3D("bracingPositionWeights", ReferenceFrame.getWorldFrame(), registry);
      bracingOrientationWeights = new YoFrameVector3D("bracingOrientationWeights", ReferenceFrame.getWorldFrame(), registry);

      bracingPositionWeights.set(4.0, 4.0, 4.0);
      bracingOrientationWeights.set(0.0, 0.0, 0.0);

      bracingFeedbackGains = new DefaultYoPIDSE3Gains("PosReactiveBracing", GainCoupling.XYZ, false, registry);
      configureGains();

      trajectoryCommand.setExecutionMode(ExecutionMode.OVERRIDE);
      trajectoryCommand.setUseCustomControlFrame(true);
      trajectoryCommand.setTrajectoryFrame(ReferenceFrame.getWorldFrame());
      controlFrame.getTransformToDesiredFrame(trajectoryCommand.getControlFramePose(), bodyToControl.getBodyFixedFrame());
   }

   public void setBracingPoint(Point3DReadOnly bracingPoint, Vector3DReadOnly bracingNormal, double trajectoryDuration)
   {
      this.desiredPosition.set(bracingPoint);
      this.bracingPlane.set(bracingPoint, bracingNormal);
      this.trajectoryDuration.set(trajectoryDuration);
   }

   @Override
   public void onEntry()
   {
      terminalVelocity.setAndScale(-terminalHandSpeed, bracingPlane.getNormal());

      trajectoryCommand.getTrajectoryPointList().clear();
      trajectoryCommand.addTrajectoryPoint(trajectoryDuration.getValue(), desiredPosition, terminalVelocity);
      positionControlHelper.handleTrajectoryCommand(trajectoryCommand, null);

      defaultPositionGains = positionControlHelper.getGains();
      defaultPositionWeights = positionControlHelper.getDefaultWeight();
      positionControlHelper.setWeights(bracingPositionWeights);

      positionControlHelper.setGains(bracingFeedbackGains.getPositionGains());
   }

   @Override
   public void doAction(double timeInState)
   {
      positionControlHelper.doAction(timeInState);
   }

   @Override
   public void onExit(double timeInState)
   {
      positionControlHelper.setGains(defaultPositionGains);
      positionControlHelper.setWeights(defaultPositionWeights);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      // sim
      double epsilon = 0.005;

      // real robot
//      double epsilon = 0.01;

      return bracingPlane.distance(positionControlHelper.getYoCurrentPosition()) < epsilon;
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
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
