package us.ihmc.exampleSimulations.simpleArm;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleArmSimulation
{
   private static final double dt = 0.001;
   private final SimulationConstructionSet scs;

   public SimpleArmSimulation()
   {
      SimpleArmRobot simpleArmRobot = new SimpleArmRobot(0.0);
      SimpleRobotInputOutputMap robotInputOutputMap = new SimpleRobotInputOutputMap(simpleArmRobot);
      DoubleYoVariable time = simpleArmRobot.getYoTime();
      RigidBody endEffectorBody = simpleArmRobot.getEndEffectorBody();
      SimpleArmController controller = new SimpleArmController(robotInputOutputMap, endEffectorBody, time);
      simpleArmRobot.setController(controller);

      SimpleArmEstimator estimator = new SimpleArmEstimator(robotInputOutputMap, dt);
      simpleArmRobot.setController(estimator);

      scs = new SimulationConstructionSet(simpleArmRobot);
      scs.setDT(dt, 1);
      scs.startOnAThread();
      scs.simulate(4.0);
   }

   public static void main(String[] args)
   {
      new SimpleArmSimulation();
   }
}
