package us.ihmc.avatar.jumpingSimulation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class JumpingSimulationFactory
{
   private final DRCRobotModel robotModel;
   private static final double gravityZ = 9.81;
   private static final String parameterResourceName = "/us/ihmc/atlas/parameters/jumping_controller.xml";

   public JumpingSimulationFactory(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public void createSimulation()
   {
      HumanoidFloatingRootJointRobot humanoidRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      JumpingSimulationController simulationController = new JumpingSimulationController(robotModel, humanoidRobot, graphicsListRegistry, gravityZ,
                                                                                         getClass().getResourceAsStream(parameterResourceName));
      humanoidRobot.setController(simulationController);

      SimulationConstructionSet scs = new SimulationConstructionSet(humanoidRobot);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();
      scs.simulate();
   }

}
