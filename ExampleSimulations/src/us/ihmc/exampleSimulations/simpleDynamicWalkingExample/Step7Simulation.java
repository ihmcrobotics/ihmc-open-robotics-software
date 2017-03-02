package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

public class Step7Simulation
{

   /**
    *  Walking robot with plotter panel & improved control -- Can walk at different speeds and feet follow parabolic trajectories during swing
    */
   
   //Variables 
   private SimulationConstructionSet sim;
   private double deltaT = 1e-4;
   private int recordFrequency = 10;
   
   public Step7Simulation()
   {
      // Graphics
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      //Robot
      Step7IDandSCSRobot_pinKnee v7Robot = new Step7IDandSCSRobot_pinKnee();
      yoGraphicsListRegistry.registerArtifactList(v7Robot.getArtifactList());
      yoGraphicsListRegistry.registerYoGraphicsList(v7Robot.getYoGraphicsList());
      
      //Controller 
      Step7WalkingController v7Controller = new Step7WalkingController(v7Robot, "v7Robot", deltaT);
      yoGraphicsListRegistry.registerArtifactList(v7Controller.getArtifactList());
      yoGraphicsListRegistry.registerYoGraphicsList(v7Controller.getYoGraphicsList());
      v7Robot.setController(v7Controller);
      
      //Simulation Object        
      sim = new SimulationConstructionSet(v7Robot);
      sim.setGroundVisible(true);
      sim.setDT(deltaT, recordFrequency);
      sim.setCameraFix(0.0, 0.0, 0.64);
      sim.setCameraPosition(0.4, -3.77, 0.56);
      sim.changeBufferSize(16000);
      sim.selectConfiguration("step7Config.guiConf");
      sim.setCameraTracking(true, true, true, false);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = sim.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      
      sim.startOnAThread();

   }

   public static void main(String[] args)
   {
      new Step7Simulation();
   }

}
