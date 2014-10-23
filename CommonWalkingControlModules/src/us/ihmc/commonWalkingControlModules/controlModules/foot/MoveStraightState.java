package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.CurrentConfigurationProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoSE3ConfigurationProvider;


public class MoveStraightState extends AbstractUnconstrainedState
{
   private StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator;
   private OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final YoSE3ConfigurationProvider finalConfigurationProvider;

   public MoveStraightState(DoubleProvider footTrajectoryTimeProvider, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, int jacobianId, DoubleYoVariable nullspaceMultiplier,
         BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule, YoSE3PIDGains gains, RobotSide robotSide,
         YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_STRAIGHT, accelerationControlModule, momentumBasedController, contactableBody, jacobianId, nullspaceMultiplier,
            jacobianDeterminantInRange, doSingularityEscape, legSingularityAndKneeCollapseAvoidanceControlModule, gains, robotSide, registry);

      RigidBody rigidBody = contactableBody.getRigidBody();
      String namePrefix = rigidBody.getName();

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "MoveStraightFootFinal", worldFrame, registry);

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();

      // TODO Check if it is necessary to implement a initial position provider using the previous desired instead of the current. (Sylvain)
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      CurrentConfigurationProvider initialConfigurationProvider = new CurrentConfigurationProvider(footFrame);

      positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix + "FootPosition", worldFrame, footTrajectoryTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "Orientation", worldFrame, footTrajectoryTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);
   }

   public void setFootPose(FramePose footPose)
   {
      finalConfigurationProvider.setPose(footPose);
   }

   @Override
   protected void initializeTrajectory()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   protected void computeAndPackTrajectory()
   {
      positionTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.compute(getTimeInCurrentState());

      positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      orientationTrajectoryGenerator.packAngularData(trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }
}