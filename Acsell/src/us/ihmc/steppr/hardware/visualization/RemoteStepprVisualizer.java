package us.ihmc.steppr.hardware.visualization;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.steppr.hardware.StepprDashboard;

public class RemoteStepprVisualizer extends SCSVisualizer
{
   public RemoteStepprVisualizer(int bufferSize)
   {
      super(bufferSize);
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      StepprDashboard.createDashboard(scs, registry);
   }
   
   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new RemoteStepprVisualizer(16384);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);
      
      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "remote");
      client.start();
   }
}
