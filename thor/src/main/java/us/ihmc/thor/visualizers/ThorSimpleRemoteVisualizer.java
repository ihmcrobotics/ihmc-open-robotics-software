package us.ihmc.thor.visualizers;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ThorSimpleRemoteVisualizer implements SCSVisualizerStateListener
{

   public ThorSimpleRemoteVisualizer(int bufferSize)
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(bufferSize);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.setShowOverheadView(true);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.start();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      PrintTools.info(this, "Starting pilot alarm service");
   }

   public static void main(String[] args)
   {
      new ThorSimpleRemoteVisualizer(32169);
   }
}
