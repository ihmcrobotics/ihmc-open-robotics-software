package us.ihmc.quadrupedRobotics;

import java.io.IOException;

import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.factories.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedTestAdministratorFactory
{
   // Factories
   private QuadrupedSimulationFactory simulationFactory;
   
   // Parameters
   private QuadrupedControlMode controlMode;
   private QuadrupedGroundContactModelType groundContactModelType;
   
   // Creation
   
   public QuadrupedTestAdministrator createTestAdministrator() throws IOException
   {
      simulationFactory.setControlMode(controlMode);
      simulationFactory.setGroundContactModelType(groundContactModelType);
      SimulationConstructionSet scs = simulationFactory.createSimulation();
      
      return new QuadrupedTestAdministrator(scs);
   }
   
   // Setters
   
   public void setSimulationFactory(QuadrupedSimulationFactory simulationFactory)
   {
      this.simulationFactory = simulationFactory;
   }
   
   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode = controlMode;
   }
   
   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType = groundContactModelType;
   }
}
