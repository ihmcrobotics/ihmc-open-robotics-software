package us.ihmc.exampleSimulations.flyballGovernor;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class FlyballGovernorSimulation
{
   private SimulationConstructionSet sim;

   public FlyballGovernorSimulation()
   {
      FlyballGovernorCommonControllerParameters controllerParameters = new FlyballGovernorCommonControllerParameters();
      
      FlyballGovernorRobot flyballGovernor1 = new FlyballGovernorRobot("UsualConstraint", controllerParameters, new Vector3D());
      FlyballGovernorRobot flyballGovernor2 = new FlyballGovernorRobot("IntegratedConstraint", controllerParameters, new Vector3D(0.5, 0.0, 0.0));

      flyballGovernor1.setController(new FlyballGovernorSimpleClosedLoopConstraintController(flyballGovernor1));

      YoVariableRegistry robotsRegistry = flyballGovernor2.getRobotsYoVariableRegistry();
      new FlyballGovernorClosedLoopConstraintToIntegrate("Constraint1", flyballGovernor2.getConstraint1A(), flyballGovernor2.getConstraint1B(), flyballGovernor2, robotsRegistry);
      new FlyballGovernorClosedLoopConstraintToIntegrate("Constraint2", flyballGovernor2.getConstraint2A(), flyballGovernor2.getConstraint2B(), flyballGovernor2, robotsRegistry);
      
      sim = new SimulationConstructionSet(new Robot[]{flyballGovernor1, flyballGovernor2});

      sim.addYoVariableRegistry(controllerParameters.getYoVariableRegistry());

      sim.setCameraPosition(0.25, 1.5, 0.2);
      sim.setCameraFix(0.25, 0.0, 0.15);

      sim.setupEntryBox("q_d_cylinder_z");
      sim.setupEntryBox("k_feedback");

      sim.setupGraph("qd_rotation");
      sim.setupGraph(new String[] {"q_d_cylinder_z", "q_cylinder_z"});
      sim.setupGraph("tau_rotation");

      sim.setDT(0.002, 10);
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new FlyballGovernorSimulation();
   }
}
