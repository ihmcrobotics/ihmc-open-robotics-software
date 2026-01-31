package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

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
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicLoadBearingPreContactState implements DynamicLoadBearingState
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
   private final YoDouble distanceToPlane;

   private final YoFramePoint3D yoBracingPoint;
   private final YoFrameVector3D yoBracingNormal;

   private final EuclideanTrajectoryControllerCommand trajectoryCommand = new EuclideanTrajectoryControllerCommand();

   public DynamicLoadBearingPreContactState(RigidBodyBasics bodyToControl,
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

      bracingFeedbackGains = new DefaultYoPIDSE3Gains("PosDynamicLoadBearing", GainCoupling.XYZ, false, registry);
      configureGains();

      distanceToPlane = new YoDouble("distanceToPlane", registry);

      trajectoryCommand.setExecutionMode(ExecutionMode.OVERRIDE);
      trajectoryCommand.setUseCustomControlFrame(true);
      trajectoryCommand.setTrajectoryFrame(ReferenceFrame.getWorldFrame());
      controlFrame.getTransformToDesiredFrame(trajectoryCommand.getControlFramePose(), bodyToControl.getBodyFixedFrame());

      yoBracingPoint = new YoFramePoint3D(bodyToControl.getName() + "BracingPoint", ReferenceFrame.getWorldFrame(), registry);
      yoBracingNormal = new YoFrameVector3D(bodyToControl.getName() + "BracingNormal", ReferenceFrame.getWorldFrame(), registry);
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

      // real robot TODO tune up
//      double epsilon = 0.01;

      distanceToPlane.set(bracingPlane.distance(positionControlHelper.getYoCurrentPosition()));
      return distanceToPlane.getValue() < epsilon;
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
      return group;
   }
}
