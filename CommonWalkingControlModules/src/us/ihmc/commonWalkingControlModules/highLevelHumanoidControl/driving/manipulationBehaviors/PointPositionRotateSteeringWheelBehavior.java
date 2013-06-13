package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.manipulationBehaviors;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.EuclideanPositionController;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.Task;
import us.ihmc.commonWalkingControlModules.trajectories.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;

/**
 * @author twan
 *         Date: 6/12/13
 */
public class PointPositionRotateSteeringWheelBehavior
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoVariableDoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider("rotateSteeringWheelTrajectoryTime", registry);
   private final YoVariableDoubleProvider desiredRotationAngleProvider = new YoVariableDoubleProvider("rotateSteeringWheelDeltaAngle", registry);

   private final EuclideanPositionController positionController;

   private final double averageAngularVelocity = 1.0;

   private final IndividualHandControlModule individualHandControlModule;
   private final ReferenceFrame creepyGripHandPositionControlFrame;

   private final PositionTrajectoryGenerator trajectoryGenerator;
   private final RobotSide robotSide;
   private final GeometricJacobian jacobian;

   public PointPositionRotateSteeringWheelBehavior(RobotSide robotSide, IndividualHandControlModule individualHandControlModule, ReferenceFrame creepyGripHandPositionControlFrame,
                                      FullRobotModel fullRobotModel, ReferenceFrame steeringWheelFrame)
   {

      this.individualHandControlModule = individualHandControlModule;
      this.creepyGripHandPositionControlFrame = creepyGripHandPositionControlFrame;
      this.robotSide = robotSide;

      RigidBody hand = fullRobotModel.getHand(robotSide);
      jacobian = new GeometricJacobian(fullRobotModel.getElevator(), hand, fullRobotModel.getElevator().getBodyFixedFrame());

      positionController = new EuclideanPositionController("pointRotateSteeringWheel", creepyGripHandPositionControlFrame, registry);
      double kp = 100.0;
      double kd = GainCalculator.computeDerivativeGain(kp, 1.0);
      positionController.setProportionalGains(kp, kp, kp);
      positionController.setDerivativeGains(kd, kd, kd);

      SE3ConfigurationProvider currentDesiredConfigurationProvider =
            individualHandControlModule.getCurrentDesiredConfigurationProvider(creepyGripHandPositionControlFrame);
      PositionProvider initialPositionProvider = currentDesiredConfigurationProvider;
      trajectoryGenerator = new CirclePositionTrajectoryGenerator("rotateSteeringWheelTrajectory", steeringWheelFrame, trajectoryTimeProvider, initialPositionProvider, registry, desiredRotationAngleProvider);
   }

   public Task getTask(double relativeRotationAngle)
   {
      return new RotateTask(relativeRotationAngle);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   private class RotateTask implements Task
   {
      private final double rotationAngle;
      private final FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame());

      private RotateTask(double rotationAngle)
      {
         this.rotationAngle = rotationAngle;
      }

      public void doTransitionIntoAction()
      {
         point.setToZero(creepyGripHandPositionControlFrame);

         double trajectoryTime = Math.abs(rotationAngle / averageAngularVelocity);
         trajectoryTimeProvider.set(trajectoryTime);
         desiredRotationAngleProvider.set(rotationAngle);
         individualHandControlModule.executePointPositionTrajectory(trajectoryGenerator, positionController, point, jacobian);
      }

      public void doAction()
      {
      }

      public void doTransitionOutOfAction()
      {
      }

      public boolean isDone()
      {
         return individualHandControlModule.isDone();
      }
   }
}
