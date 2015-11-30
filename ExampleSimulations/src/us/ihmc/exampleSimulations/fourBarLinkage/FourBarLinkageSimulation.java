package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class FourBarLinkageSimulation
{
   private Vector3d offsetWorld = new Vector3d(0.0, 0.0, 0.0);
   private static final double simDT = 1.0e-4;
   private static final int recordFrequency = 10;

   public FourBarLinkageSimulation(FourBarLinkageParameters fourBarLinkageParameters)
   {
      YoVariableRegistry registry = new YoVariableRegistry("fourBarLinkageRegistry");

      // Robot
      FourBarLinkageRobot robot = new FourBarLinkageRobot("basicFourBar", fourBarLinkageParameters, offsetWorld, registry);

      // Pinning joint 2 to world
      FourBarLinkageController controller = new FourBarLinkageController(robot, "fourBarLinkageController", fourBarLinkageParameters);
      controller.setGain(1000.0);
      controller.setDamping(100.0);
      robot.setController(controller);

      // SCS
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      //      scs.setSimulateDuration(3.0);
      scs.setGroundVisible(false);
      scs.setDT(simDT, recordFrequency);
      scs.addYoVariableRegistry(registry);
      scs.setCameraFix(0.335, 0.0, -0.5);
      scs.setCameraPosition(0.355, -11.2, -0.75);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      FourBarLinkageParameters fourBarLinkageParameters = new FourBarLinkageParameters();
      fourBarLinkageParameters.linkageLength_1 = fourBarLinkageParameters.linkageLength_3 = 1.0;
      fourBarLinkageParameters.linkageLength_2 = fourBarLinkageParameters.linkageLength_4 = 2.0;
      fourBarLinkageParameters.damping_1 = fourBarLinkageParameters.damping_2 = fourBarLinkageParameters.damping_3 = 0.5;
      fourBarLinkageParameters.mass_1 = fourBarLinkageParameters.mass_2 = fourBarLinkageParameters.mass_3 = fourBarLinkageParameters.mass_4 = 0.3;
      fourBarLinkageParameters.radius_1 = fourBarLinkageParameters.radius_2 = fourBarLinkageParameters.radius_3 = fourBarLinkageParameters.radius_4 = 0.03;
      fourBarLinkageParameters.initial_angle_0 = fourBarLinkageParameters.initial_angle_1 = fourBarLinkageParameters.initial_angle_2 = 0.5 * Math.PI;

      new FourBarLinkageSimulation(fourBarLinkageParameters);
   }
}