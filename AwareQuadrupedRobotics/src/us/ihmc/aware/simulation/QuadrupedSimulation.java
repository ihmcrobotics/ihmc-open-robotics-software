package us.ihmc.aware.simulation;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedSimulation
{
   private static final double SIMULATE_DT = 0.0003;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATE_DT);

   private static final boolean USE_GPU_LIDAR = false;
   private static final boolean SHOW_OVERHEAD_VIEW = false;
   private static final double GRAVITY = -9.81;

   private final SimulationConstructionSet simulation;
   private final SDFRobot sdfRobot;
   private final GroundContactModel contactModel;

   public QuadrupedSimulation(SDFRobot sdfRobot, SimulationConstructionSetParameters parameters,
         YoGraphicsListRegistry graphicsListRegistry, YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead)
   {
      this.sdfRobot = sdfRobot;
      sdfRobot.setGravity(GRAVITY);

      this.contactModel = createContactModel(sdfRobot);
      sdfRobot.setGroundContactModel(contactModel);

      if (parameters == null)
      {
         parameters = new SimulationConstructionSetParameters();
      }
      this.simulation = createSimulation(sdfRobot, parameters);

      setupGraphics(simulation, graphicsListRegistry, graphicsListRegistryForDetachedOverhead);

      if (USE_GPU_LIDAR)
      {
         // TODO: implement
      }
   }

   public void start()
   {
      simulation.startOnAThread();
      simulation.simulate();
   }

   public SimulationConstructionSet getSimulation()
   {
      return simulation;
   }

   public SDFRobot getSdfRobot()
   {
      return sdfRobot;
   }

   public GroundContactModel getContactModel()
   {
      return contactModel;
   }

   private GroundContactModel createContactModel(SDFRobot sdfRobot)
   {
      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(sdfRobot,
            sdfRobot.getRobotsYoVariableRegistry());
      GroundProfile3D groundProfile = new FlatGroundProfile(0.0);
      groundContactModel.setGroundProfile3D(groundProfile);
      groundContactModel.setZStiffness(2500.0);
      groundContactModel.setZDamping(250.0);
      groundContactModel.setXYStiffness(2500.0);
      groundContactModel.setXYDamping(250.0);

      return groundContactModel;
   }

   private SimulationConstructionSet createSimulation(SDFRobot sdfRobot, SimulationConstructionSetParameters parameters)
   {
      SimulationConstructionSet simulation = new SimulationConstructionSet(sdfRobot, parameters);
      simulation.setDT(SIMULATE_DT, RECORD_FREQUENCY);

      return simulation;
   }

   private void setupGraphics(SimulationConstructionSet simulation, YoGraphicsListRegistry graphicsListRegistry,
         YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead)
   {
      simulation.addYoGraphicsListRegistry(graphicsListRegistry);
      graphicsListRegistry.setYoGraphicsUpdatedRemotely(true);

      VisualizerUtils.createOverheadPlotter(simulation, SHOW_OVERHEAD_VIEW, "centerOfMass", graphicsListRegistry);
      VisualizerUtils.createOverheadPlotterInSeparateWindow(simulation, SHOW_OVERHEAD_VIEW, "centerOfMass",
            graphicsListRegistryForDetachedOverhead);

      simulation.setCameraTrackingVars("bodyCoMX", "bodyCoMY", "bodyCoMZ");
      simulation.setCameraDollyVars("bodyCoMX", "bodyCoMY", "bodyCoMZ");
   }

   private void setupJoystick(YoVariableHolder registry, PacketCommunicator controllerCommunicator)
   {
      try
      {
         // TODO: implement
      }
      catch (Exception exception)
      {
         System.err
               .println("Couldn't find a Joystick or problem mapping buttons to the joystick. Proceeding without one!");
      }
   }
}
