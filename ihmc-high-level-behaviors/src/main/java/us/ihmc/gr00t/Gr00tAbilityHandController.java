package us.ihmc.gr00t;

import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

/** Ability Hand implementation of the reusable GR00T hand contract. */
public final class Gr00tAbilityHandController implements Gr00tHandController
{
   public static final int TARGET_COUNT = 6;
   private static final float MAX_ACTUATOR_VELOCITY_DEGREES = 30.0f;
   private final SideDependentList<AbilityHandActionComms> hands;
   private final SideDependentList<AtomicReference<Boolean>> gripRequests = new SideDependentList<>(side -> new AtomicReference<>());
   private final Consumer<String> statusUpdater;
   private volatile boolean enabled;

   public Gr00tAbilityHandController(ROS2Node ros2Node, Consumer<String> statusUpdater)
   {
      this.statusUpdater = statusUpdater == null ? status -> { } : statusUpdater;
      hands = new SideDependentList<>(side -> new AbilityHandActionComms(side, ros2Node));
   }

   @Override
   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         AbilityHandActionComms hand = hands.get(side);
         hand.update();
         Boolean close = gripRequests.get(side).getAndSet(null);
         if (close == null)
            continue;
         if (!hand.isConnected())
         {
            gripRequests.get(side).compareAndSet(null, close);
            statusUpdater.accept("Waiting for " + side.getLowerCaseName() + " Ability Hand before sending grip command");
            continue;
         }
         AbilityHandCommand command = hand.getCommand();
         command.setControlMode(AbilityHandCommand.GRIP_CONTROL);
         command.setGrip(close ? AbilityHandCommand.CLOSE_GRIP : AbilityHandCommand.OPEN_GRIP);
         hand.publishCommand();
         statusUpdater.accept(side.getPascalCaseName() + (close ? " hand close command sent" : " hand open command sent"));
      }
   }

   @Override
   public boolean publishPolicyTargets(RobotSide side, double[] targetsRadians)
   {
      AbilityHandActionComms hand = hands.get(side);
      if (!enabled || !hand.isConnected() || targetsRadians.length < TARGET_COUNT)
         return false;

      AbilityHandCommand command = hand.getCommand();
      command.setControlMode(AbilityHandCommand.POSITION_CONTROL);
      float[] goals = command.getGoalPositions();
      for (int actuator = 0; actuator < 4; actuator++)
         goals[actuator] = (float) Math.toDegrees(targetsRadians[actuator]);
      goals[4] = (float) Math.toDegrees(targetsRadians[5]);
      goals[5] = (float) Math.toDegrees(targetsRadians[4]);
      java.util.Arrays.fill(command.getGoalVelocities(), MAX_ACTUATOR_VELOCITY_DEGREES);
      hand.publishCommand();
      return true;
   }

   @Override
   public boolean hasState(RobotSide side)
   {
      return hands.get(side).isConnected();
   }

   @Override
   public double[] getJointPositions(RobotSide side)
   {
      double[] positions = new double[TARGET_COUNT];
      AbilityHandState state = hands.get(side).getLatestState();
      if (state == null)
         return positions;
      float[] actuatorPositionsDegrees = state.getActuatorPositions();
      for (int actuator = 0; actuator < 4; actuator++)
         positions[actuator] = Math.toRadians(actuatorPositionsDegrees[actuator]);
      positions[4] = Math.toRadians(actuatorPositionsDegrees[5]);
      positions[5] = Math.toRadians(actuatorPositionsDegrees[4]);
      return positions;
   }

   @Override
   public void requestGrip(RobotSide side, boolean close)
   {
      gripRequests.get(side).set(close);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
      if (!enabled)
         gripRequests.forEach(request -> request.set(null));
   }

   @Override
   public boolean isEnabled()
   {
      return enabled;
   }
}
