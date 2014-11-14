package us.ihmc.atlas.velocityControlEvaluation;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class VelocityControlEvaluationSimulation
{
   
   public VelocityControlEvaluationSimulation()
   {
      double simDT = 0.001;
      double controlDT = 0.006;
      int ticksPerControl = (int) Math.round(controlDT/simDT);
      controlDT = simDT * ticksPerControl;
      
      System.out.println("controlDT = " + controlDT);
      
      VelocityControlEvaluationRobot robot = new VelocityControlEvaluationRobot();
      VelocityControlEvaluationController controller = new VelocityControlEvaluationController(robot, controlDT);
      robot.setController(controller, ticksPerControl);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, 32768);
      scs.setDT(simDT, 1);
      scs.setSimulateDuration(1.5);
      
      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      new VelocityControlEvaluationSimulation();
   }
}
