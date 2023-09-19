package us.ihmc.valkyrie.visualizer;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DiagnosticRemoteValkyrieVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;
   
   public DiagnosticRemoteValkyrieVisualizer()
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(BUFFER_SIZE);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.startWithHostSelector();
   }
   

   @Override
   public void starting(final SimulationConstructionSet scs, Robot robot, YoRegistry registry)
   {

   }
   
   public static void main(String[] args)
   {
      new DiagnosticRemoteValkyrieVisualizer();
   }
}
