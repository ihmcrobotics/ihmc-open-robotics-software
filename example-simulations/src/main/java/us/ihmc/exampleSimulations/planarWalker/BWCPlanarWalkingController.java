package us.ihmc.exampleSimulations.planarWalker;

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

   private final SideDependentList<PDController> kneeLengthControllers = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredKneeForces = new SideDependentList<>();

   private final YoDouble desiredLegLength = new YoDouble("desiredLegLength", registry);

   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot)
   {
      this.controllerRobot = controllerRobot;

      // set up our leg length gains for the leg length controller
      YoDouble legLengthKp = new YoDouble("legLengthKp", registry);
      YoDouble legLengthKd = new YoDouble("legLengthKd", registry);
      legLengthKp.set(1500.0);
      legLengthKd.set(50.0);

      desiredLegLength.set((BWCPlanarWalkingRobotDefinition.shinLength + BWCPlanarWalkingRobotDefinition.thighLength) / 2.0);

      // set up our leg length controllers
      PDController leftLegLengthController = new PDController(legLengthKp, legLengthKd, "leftLegLengthController", registry);
      PDController rightLegLengthController = new PDController(legLengthKp, legLengthKd, "rightLegLengthController", registry);
      YoDouble leftLegDesiredKneeForce = new YoDouble("leftLegDesiredKneeForce", registry);
      YoDouble rightLegDesiredKneeForce = new YoDouble("rightLegDesiredKneeForce", registry);
      kneeLengthControllers.put(RobotSide.LEFT, leftLegLengthController);
      kneeLengthControllers.put(RobotSide.RIGHT, rightLegLengthController);
      desiredKneeForces.put(RobotSide.LEFT, leftLegDesiredKneeForce);
      desiredKneeForces.put(RobotSide.RIGHT, rightLegDesiredKneeForce);

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

      for (RobotSide robotSide : RobotSide.values)
      {
         double legLength = controllerRobot.getLegLength(robotSide);
         double kneeVelocity = -controllerRobot.getKneeJoint(robotSide).getQd();

         double desiredLegLength = this.desiredLegLength.getDoubleValue();
         double desiredKneeVelocity = 0.0;
         // compute and record the desired knee force to hold the leg at the desired length
         double desiredKneeForce = kneeLengthControllers.get(robotSide).compute(legLength, desiredLegLength, kneeVelocity, desiredKneeVelocity);
         desiredKneeForces.get(robotSide).set(desiredKneeForce);

         // set the desired force to the knee joint to hold the leg at the desired length
         controllerRobot.getKneeJoint(robotSide).setTau(-desiredKneeForce);
      }
   }
}
