package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class RobotArmSimulation
{
   public static void main(String[] args)
   {
      double controlDT = 5.0e-5;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RobotArm robotArm = new RobotArm();
      RobotArmController robotArmController = new RobotArmController(robotArm, controlDT, yoGraphicsListRegistry);
      robotArm.setController(robotArmController);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize((int) Math.pow(2, 16)); // => 65536
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArm, parameters);
      scs.setFastSimulate(true, 15);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setDT(controlDT, 10);
      scs.startOnAThread();
   }
}
