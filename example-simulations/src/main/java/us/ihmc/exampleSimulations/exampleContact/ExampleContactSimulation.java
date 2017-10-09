package us.ihmc.exampleSimulations.exampleContact;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;
import us.ihmc.exampleSimulations.pointMassRobot.PointMassRobot;

public class ExampleContactSimulation
{
   public ExampleContactSimulation()
   {
      double forceVectorScale = 1.0/50.0;
      
//      ContactableSphereRobot contactableRobot = new ContactableSphereRobot();
      ContactableSelectableBoxRobot contactableRobot = new ContactableSelectableBoxRobot();
      contactableRobot.setPosition(0.0, 0.0, 0.5);
      contactableRobot.createAvailableContactPoints(1, 1, forceVectorScale, true);
      contactableRobot.setGravity(0.0);
      
//      PushStickRobot pushStickRobot = new PushStickRobot();
//      PushStickController pushStickController = new PushStickController(pushStickRobot.getExternalForcePoint(), contactableSphereRobot);
//      pushStickRobot.setController(pushStickController);
//      
//      Robot[] robots = new Robot[]{contactableSphereRobot, pushStickRobot};

      PointMassRobot pointMassRobot = new PointMassRobot();
      pointMassRobot.setPosition(-0.6, -0.1, 0.3);
      pointMassRobot.setVelocity(0.1, 0.25, 0.0);
      
      ContactController contactController = new ContactController();
      contactController.addContactPoint(pointMassRobot.getExternalForcePoint());
      contactController.addContactable(contactableRobot);
      
      pointMassRobot.setController(contactController);
      
      Robot[] robots = new Robot[]{contactableRobot, pointMassRobot};
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(36000);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robots, parameters);
      scs.setDT(0.001, 1);
      scs.setSimulateDuration(8.0);
      
      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      new ExampleContactSimulation();
   }
}
