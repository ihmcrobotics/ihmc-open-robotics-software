package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.CurrentConfigurationProvider;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class MoveStraightState extends AbstractUnconstrainedState
{
   private StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator;
   private OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   public MoveStraightState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_STRAIGHT, footControlHelper, gains, registry);

      RigidBody rigidBody = contactableFoot.getRigidBody();
      String namePrefix = rigidBody.getName();

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "MoveStraightFootFinal", worldFrame, registry);
      trajectoryTimeProvider = new YoVariableDoubleProvider(namePrefix + "MoveStraightTrajectoryTime", registry);

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();

      // TODO Check if it is necessary to implement a initial position provider using the previous desired instead of the current. (Sylvain)
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      CurrentConfigurationProvider initialConfigurationProvider = new CurrentConfigurationProvider(footFrame);

      positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix + "FootPosition", worldFrame, trajectoryTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "Orientation", worldFrame, trajectoryTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);
   }

   public void setFootPose(FramePose footPose, double trajectoryTime)
   {
      finalConfigurationProvider.setPose(footPose);
      trajectoryTimeProvider.set(trajectoryTime);
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
      orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }
}