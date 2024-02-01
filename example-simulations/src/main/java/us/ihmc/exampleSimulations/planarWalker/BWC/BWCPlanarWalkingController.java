package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final BWCPlanarWalkingRobot controllerRobot;
   private final SideDependentList<PDController> legLengthControllers = new SideDependentList<>();

   private final SideDependentList<YoDouble> desiredKneeForces = new SideDependentList<>();

   private final YoDouble desiredLegLength;

   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot)
   {
      this.controllerRobot = controllerRobot;

      YoDouble legLengthKp = new YoDouble("legLengthKp", registry);
      YoDouble legLengthKd = new YoDouble("legLengthKd", registry);
      legLengthKp.set(1500.0);
      legLengthKd.set(50.0);

      PDController leftLegLengthController = new PDController(legLengthKp, legLengthKd, "leftLegLengthController", registry);
      PDController rightLegLengthController = new PDController(legLengthKp, legLengthKd, "rightLegLengthController", registry);
      legLengthControllers.put(RobotSide.LEFT, leftLegLengthController);
      legLengthControllers.put(RobotSide.RIGHT, rightLegLengthController);

      desiredKneeForces.put(RobotSide.LEFT, new YoDouble("leftLegDesiredKneeForce", registry));
      desiredKneeForces.put(RobotSide.RIGHT, new YoDouble("rightLegDesiredKneeForce", registry));

      desiredLegLength = new YoDouble("desiredLegLength", registry);
      desiredLegLength.set((BWCPlanarWalkingRobotDefinition.shinLength + BWCPlanarWalkingRobotDefinition.thighLength) / 2);

      registry.addChild(controllerRobot.getYoRegistry());
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      controllerRobot.update();

      for (RobotSide robotSide : RobotSide.values())
      {
         double kneePosition = controllerRobot.getLegLength(robotSide);
         double kneeVelocity = -controllerRobot.getKneeJoint(robotSide).getQd();

         double desiredKneePosition = desiredLegLength.getDoubleValue();
         double desiredKneeVelocity = 0.0;

         double torque = -legLengthControllers.get(robotSide).compute(kneePosition, desiredKneePosition, kneeVelocity, desiredKneeVelocity);

         desiredKneeForces.get(robotSide).set(torque);
         controllerRobot.getKneeJoint(robotSide).setTau(torque);
      }
   }
}
