package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.ChangeableConfigurationProvider;
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

   private final ReferenceFrame footFrame;
   private final FramePose initialFootPose = new FramePose();
   private final ChangeableConfigurationProvider initialConfigurationProvider = new ChangeableConfigurationProvider();

   public MoveStraightState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_STRAIGHT, footControlHelper, gains, registry);

      RigidBody rigidBody = contactableFoot.getRigidBody();
      String namePrefix = rigidBody.getName();

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "MoveStraightFootFinal", worldFrame, registry);
      trajectoryTimeProvider = new YoVariableDoubleProvider(namePrefix + "MoveStraightTrajectoryTime", registry);

      footFrame = momentumBasedController.getReferenceFrames().getFootFrame(robotSide);

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
      if (getPreviousState() == this)
      {
         positionTrajectoryGenerator.get(desiredPosition);
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialFootPose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      }
      else
      {
         initialFootPose.setToZero(footFrame);
      }
      initialConfigurationProvider.set(initialFootPose);

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