package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;


import us.ihmc.exampleSimulations.m2.LittleM2Parameters;
import us.ihmc.exampleSimulations.m2.M2Parameters;
import us.ihmc.exampleSimulations.m2.M2ProcessedSensorsControllerWithStateTransitions;
import us.ihmc.exampleSimulations.m2.M2Robot;
import us.ihmc.exampleSimulations.m2.M2Simulation;
import us.ihmc.exampleSimulations.m2.Output.PerfectProcessedOutputs;
import us.ihmc.exampleSimulations.m2.Sensors.PerfectSensorProcessing;
import us.ihmc.exampleSimulations.m2.Sensors.ProcessedSensors;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class M2SimulationConstructor implements SimulationConstructor, SimulationDoneCriterion
{
   private static final long serialVersionUID = 4634305300496002L;
   private DoubleYoVariable t;
   private TerrainObject3D profile;

   public M2SimulationConstructor()
   {
   }

   public Simulation constructSimulation(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      M2Simulation.USE_HEAVY_M2 = false;

      M2Parameters m2Parameters = new LittleM2Parameters();
      M2Robot m2 = new M2Robot(m2Parameters);

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      GroundContactModel linearGroundModel = new LinearGroundContactModel(m2, 40000.0, 100.0, 40.0, 500.0, registry );

      // HurosotGroundProfile profile = new HurosotGroundProfile(30.0, 1.5);
//    profile = new HurosotTerrain(30.0, 1.5);
//    linearGroundModel.setGroundProfile(profile);

      m2.setGroundContactModel(linearGroundModel);

      ProcessedSensors processedSensors = new ProcessedSensors();

      PerfectSensorProcessing perfectSensorProcessing = new PerfectSensorProcessing(m2, processedSensors);
      PerfectProcessedOutputs perfectProcessedOutputs = new PerfectProcessedOutputs(m2);


      // ***jjc twan added a getName method to RobotControllers, but i do not know what name should be passed in, sending in null for now.
      M2ProcessedSensorsControllerWithStateTransitions m2Controller = new M2ProcessedSensorsControllerWithStateTransitions(m2Parameters, processedSensors,
                                                                         perfectSensorProcessing, perfectProcessedOutputs, null);
      m2.setController(m2Controller);

      Simulation m2Sim = new Simulation(m2, 1);

      m2Sim.setDT(0.0002, 100);

      m2Sim.setSimulateDoneCriterion(this);

      t = (DoubleYoVariable) m2Sim.getVariable("t");

      return m2Sim;
   }

   public void setupGraphics(Simulation m2Sim)
   {
      System.err.println("Need to implement this again for SimulationDispatcher!!!");

//      m2Sim.setupSimulationGraphics(graphicsSynchronizer, null);
//
//      if (profile != null)
//      {
//         m2Sim.getSimulationGraphics().addStaticLinkGraphics(profile.getLinkGraphics());
//         m2Sim.getSimulationGraphics().setGroundVisible(false);
//      }

      // m2Sim.getSimulationGraphics().addStaticLink(profile.getLink());
      // m2Sim.getSimulationGraphics().setClipDistances(1.0, 100.0);  //(0.15, 20.0);
      // m2Sim.getSimulationGraphics().setGroundVisible(false);

      // m2Sim.getSimulationGraphics().setCameraFix(1.3, -21.7, 1.3);
      // m2Sim.getSimulationGraphics().setCameraPosition(13.6, -36.4, 4.85);

      // m2Sim.getSimulationGraphics().setCameraTracking(false, true, true, false);
      // m2Sim.getSimulationGraphics().setCameraDolly(false, true, true, false);

   }


   public boolean isSimulationDone()
   {
      if (t.getDoubleValue() > 30.0)
         return true;
      else
         return false;
   }

   public void setupAndLaunchSCS()
   {
      Simulation m2Simulation = constructSimulation(null, null);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      SimulationConstructionSet scs = new SimulationConstructionSet(m2Simulation, parameters);
      if (profile != null)
      {
         scs.addStaticLinkGraphics(profile.getLinkGraphics());
         scs.setGroundVisible(false);
      }

      Thread thread = new Thread(scs);
      thread.start();

   }
   
   
   public static void main(String[] args)
   {
      M2SimulationConstructor m2SimulationConstructor = new M2SimulationConstructor();
      m2SimulationConstructor.setupAndLaunchSCS();
   }

   public void doActionAfterSimulationStateInitialized(Simulation simulation)
   {
      // TODO Auto-generated method stub
      
   }
}
