package us.ihmc.avatar.jumpingSimulation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class JumpingSimulationFactory
{
   private final DRCRobotModel robotModel;
   private static final double gravityZ = -9.81;

   public JumpingSimulationFactory(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public void createSimulation()
   {
      HumanoidFloatingRootJointRobot humanoidRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      JumpingSimulationController simulationController = new JumpingSimulationController(robotModel, humanoidRobot, graphicsListRegistry, gravityZ);
      humanoidRobot.setController(simulationController);

      SimulationConstructionSet scs = new SimulationConstructionSet(humanoidRobot);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();
   }

}
