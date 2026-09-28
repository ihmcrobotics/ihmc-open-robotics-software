package us.ihmc.avatar.scs2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.kinematicsSimulation.AbilityHandKinematicsSimulation;
import us.ihmc.handsros2.HandType;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * SCS2 robot controller that kinematically drives Ability Hand finger joints from the same ROS 2
 * command/state API as {@link AbilityHandKinematicsSimulation}.
 * <p>
 * Finger joints stay on {@code nameOfJointsToIgnore} so they are not merged into the walking /
 * estimator DoFs. This controller writes their configuration each tick; the physics engine copies
 * that onto the ignored joints via {@code writeControllerOutputForJointsToIgnore}.
 */
public class AbilityHandSimulationController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final SideDependentList<AbilityHandKinematicsSimulation> hands = new SideDependentList<>();

   public AbilityHandSimulationController(ControllerInput controllerInput,
                                          ControllerOutput controllerOutput,
                                          ROS2Node ros2Node,
                                          DRCRobotModel robotModel)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;

      for (RobotSide side : RobotSide.values)
      {
         if (robotModel.getRobotVersion().getHandType(side) != HandType.ABILITY_HAND)
            continue;

         AbilityHandKinematicsSimulation handSimulation = new AbilityHandKinematicsSimulation(side,
                                                                                              ros2Node,
                                                                                              this::findOneDoFJoint,
                                                                                              robotModel.getHandModel(side));
         if (handSimulation.isEnabled())
         {
            hands.put(side, handSimulation);
            LogTools.info("Ability Hand simulation enabled for {} side.", side.getLowerCaseName());
         }
         else
         {
            LogTools.warn("Ability Hand joints not found on the simulated robot for {} side; finger simulation disabled.",
                          side.getLowerCaseName());
         }
      }
   }

   public static void attachIfNeeded(Robot robot, ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      boolean hasAbilityHand = false;
      for (RobotSide side : RobotSide.values)
      {
         if (robotModel.getRobotVersion().getHandType(side) == HandType.ABILITY_HAND)
            hasAbilityHand = true;
      }
      if (!hasAbilityHand)
         return;

      robot.addController((input, output) -> new AbilityHandSimulationController(input, output, ros2Node, robotModel));
   }

   public boolean isEnabled()
   {
      return !hands.isEmpty();
   }

   @Override
   public void initialize()
   {
      doControl();
   }

   @Override
   public void doControl()
   {
      for (RobotSide side : hands.sides())
      {
         AbilityHandKinematicsSimulation hand = hands.get(side);
         hand.update(controllerInput.getTime());
         hand.writeJointOutput(controllerOutput);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private OneDoFJointBasics findOneDoFJoint(String jointName)
   {
      JointReadOnly joint = controllerInput.getInput().findJoint(jointName);
      if (joint instanceof OneDoFJointBasics oneDoFJoint)
         return oneDoFJoint;
      return null;
   }
}
