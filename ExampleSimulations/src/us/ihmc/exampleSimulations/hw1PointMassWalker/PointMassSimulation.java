package us.ihmc.exampleSimulations.hw1PointMassWalker;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class PointMassSimulation
{
   private static final int bufferSize = 32000;
   private static final double simDT = 1.0e-4;
   private static final int recordFrequency = 10;
   
   public PointMassSimulation()
   {
      // Graphics
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      // Robot
      PointMassSCSRobot robotSCS = new PointMassSCSRobot();
      
      PointMassIDRobot robotID = new PointMassIDRobot(robotSCS, robotSCS.getRobotsYoVariableRegistry());
      yoGraphicsListRegistry.registerArtifactList(robotID.getArtifactList());
      yoGraphicsListRegistry.registerYoGraphicsList(robotID.getYoGraphicsList());
          
      // Controller
      PointMassController controller = new PointMassController(robotID, robotSCS.getYoTime(), simDT);
      yoGraphicsListRegistry.registerArtifactList(controller.getArtifactList());
      yoGraphicsListRegistry.registerYoGraphicsList(controller.getYoGraphicsList());
      robotSCS.setController(controller);
      
      // SCS
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotSCS, parameters);
      scs.setDT(simDT, recordFrequency);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setCameraDollyOffsets(0.0, -3.0, -0.15);
      scs.setCameraDolly(true, true, true, false);
      scs.setCameraTracking(true, true, true, false);
      VisualizerUtils.createOverheadPlotter(scs, true, "Center of Mass", yoGraphicsListRegistry);
   
      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      new  PointMassSimulation();
   }
}
