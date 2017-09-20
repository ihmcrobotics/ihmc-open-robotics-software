package us.ihmc.exampleSimulations.agileRobotArm;

import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AgileRobotArmSimulation
{
   SimulationConstructionSet sim;

   public AgileRobotArmSimulation()
   {
      AgileRobotArmRobot agileRobotArmRobot = new AgileRobotArmRobot();

      // agileRobotArmRobot.createControllerBase(System.out, "AgileRobotArm");
      AgileRobotArmController controller = new AgileRobotArmController(agileRobotArmRobot);
      agileRobotArmRobot.setController(controller);
      sim = new SimulationConstructionSet(agileRobotArmRobot);

      setupGUI(sim);
      sim.setDT(0.0002, 100);
      sim.setFastSimulate(true);

      sim.startOnAThread();
   }

   public static void main(String[] args)
   {
      new AgileRobotArmSimulation();
   }


   public void setupGUI(SimulationConstructionSet sim)
   {
      sim.setClipDistances(0.1, 100.0);
      sim.setFieldOfView(1.0);

      sim.setPlaybackRealTimeRate(1.0);
      sim.setPlaybackDesiredFrameRate(0.033);
      sim.setGraphsUpdatedDuringPlayback(true);

      sim.setupVarGroup("kinematics", new String[] {"t"}, new String[] {"q_.*", "qd_.*"});

      sim.setupVarGroup("torques", null, new String[] {"t", "tau_.*"});

      // Set up some initial graphs:

      GraphConfiguration auto = new GraphConfiguration("auto", GraphConfiguration.AUTO_SCALING);
      GraphConfiguration manual1 = new GraphConfiguration("manual1", GraphConfiguration.MANUAL_SCALING, -1.0, 1.0);
      GraphConfiguration phase1 = new GraphConfiguration("phase1", GraphConfiguration.AUTO_SCALING);
      phase1.setPlotType(GraphConfiguration.PHASE_PLOT);

      sim.setupGraphConfigurations(new GraphConfiguration[] {auto, manual1, phase1});

      sim.setupGraphGroup("tracking", new String[][][]
      {
         {
            {"mode"}, {""}
         },
         {
            {"q_shoulder_yaw", "q_d_shoulder_yaw"}, {"auto"}
         },
         {
            {"q_shoulder_pitch", "q_d_shoulder_pitch"}, {"auto"}
         },
         {
            {"q_elbow_pitch", "q_d_elbow_pitch"}, {"auto"}
         },
         {
            {"q_wrist_pitch", "q_d_wrist_pitch"}, {"auto"}
         },
         {
            {"q_wrist_yaw", "q_d_wrist_yaw"}, {"auto"}
         },
         {
            {"q_wrist_roll", "q_d_wrist_roll"}, {"auto"}
         }
      }, 2);


      sim.setupGraphGroup("joints", new String[][]
      {
         {"q_shoulder_yaw", "qd_shoulder_yaw"}, {"q_shoulder_pitch", "qd_shoulder_pitch"}, {"q_elbow_pitch", "qd_elbow_pitch"},
         {"q_wrist_pitch", "qd_wrist_pitch"}, {"q_wrist_yaw", "qd_wrist_yaw"}, {"q_wrist_roll", "qd_wrist_roll"}
      });


      sim.setupGraphGroup("shoulder_tracking", new String[][]
      {
         {"mode"}, {"q_shoulder_yaw", "q_d_shoulder_yaw"}, {"q_shoulder_pitch", "q_d_shoulder_pitch"}, {"q_elbow_pitch", "q_d_elbow_pitch"},
         {"tau_shoulder_yaw", "tau_shoulder_pitch", "tau_elbow_pitch"}
      });

      sim.setupGraphGroup("torques", new String[][]
      {
         {"tau_shoulder_yaw"}, {"tau_shoulder_pitch"}, {"tau_elbow_pitch"}, {"tau_wrist_pitch"}, {"tau_wrist_yaw"}, {"tau_wrist_roll"},
      });



      sim.setupEntryBoxGroup("torques", new String[]
      {
         "mode", "tau_shoulder_yaw", "tau_shoulder_pitch", "tau_elbow_pitch", "tau_wrist_yaw", "tau_wrist_pitch", "tau_wrist_roll"
      });

      sim.setupEntryBoxGroup("shoulder_tracking", new String[]
      {
         "mode", "q_d_shoulder_yaw", "q_d_shoulder_pitch", "q_d_elbow_pitch", "k_shoulder_yaw", "k_shoulder_pitch", "k_elbow_pitch", "b_shoulder_yaw",
         "b_shoulder_pitch", "b_elbow_pitch"
      });

      sim.setupEntryBoxGroup("gains", new String[]
      {
         "mode", "k_shoulder_yaw", "k_shoulder_pitch", "k_elbow_pitch", "k_wrist_pitch", "k_wrist_yaw", "b_shoulder_yaw", "b_shoulder_pitch", "b_elbow_pitch",
         "b_wrist_pitch", "b_wrist_yaw"
      });

      sim.setupEntryBoxGroup("traj", new String[] {"mode", "shoulder_pitch_amp", "shoulder_pitch_freq", "shoulder_yaw_amp", "shoulder_yaw_freq"});

      sim.setupEntryBoxGroup("anti_gravity", new String[]
      {
         "mode", "a1", "a2", "a3", "a4", "a5"
      });


      sim.setupConfiguration("shoulder_tracking", "all", "shoulder_tracking", "shoulder_tracking");
      sim.setupConfiguration("joints", "all", "joints", "joint_offsets");
      sim.setupConfiguration("tracking", "all", "tracking", "gains");
      sim.setupConfiguration("torques", "all", "joints", "torques");
      sim.setupConfiguration("anti_gravity", "all", "torques", "anti_gravity");

      // sim.selectConfiguration("joints");
      sim.selectConfiguration("tracking");

      // sim.selectConfiguration("shoulder_tracking");

      sim.createNewGraphWindow("tracking");

      sim.setCameraFix(0.0, 0.0, 1.2);
      sim.setCameraPosition(7.0, -5.5, 2.1);

      sim.setCameraTracking(false, true, true, false);
   }

}
