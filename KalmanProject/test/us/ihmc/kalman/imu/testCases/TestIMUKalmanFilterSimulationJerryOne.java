package us.ihmc.kalman.imu.testCases;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.kalman.imu.QuaternionBasedArrayFullIMUKalmanFilter;
import us.ihmc.kalman.imu.QuaternionBasedFullIMUKalmanFilter;
import us.ihmc.kalman.imu.QuaternionBasedJamaFullIMUKalmanFilter;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionsettools.util.inputdevices.MidiSliderBoard;

public class TestIMUKalmanFilterSimulationJerryOne
{
   private static final boolean PROFILING = false;    // true; //

   private static final double SIMULATION_DT = 0.0001;
   private static final double KALMAN_UPDATE_DT = 0.0025;
   private static final double WINDOW = 0.0025;

   public TestIMUKalmanFilterSimulationJerryOne()
   {
      TestIMUKalmanFilterRobot rob = new TestIMUKalmanFilterRobot();

      QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilter = new QuaternionBasedJamaFullIMUKalmanFilter(KALMAN_UPDATE_DT);
      QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilterTwo = new QuaternionBasedArrayFullIMUKalmanFilter(KALMAN_UPDATE_DT,
                                                                                    rob.getRobotsYoVariableRegistry());

      TestIMUKalmanFilterControllerJerryOne controller = new TestIMUKalmanFilterControllerJerryOne(rob, PROFILING, quaternionBasedFullIMUKalmanFilter,
                                                            quaternionBasedFullIMUKalmanFilterTwo, "testIMUKalmanFilterControllerJerryOne");
      rob.setController(controller, (int) Math.round(KALMAN_UPDATE_DT / SIMULATION_DT));

      SimulationConstructionSet scs;
      if (PROFILING)
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
         parameters.setCreateGUI(false);
         parameters.setDataBufferSize(4096);
         scs = new SimulationConstructionSet(rob, parameters);
      }
      else
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setDataBufferSize(4096);
         scs = new SimulationConstructionSet(rob, parameters);
      }

      scs.setDT(SIMULATION_DT, (int) (WINDOW / SIMULATION_DT));

      Link coords = new Link("coords");
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.2);
      coords.setLinkGraphics(linkGraphics);
      scs.addStaticLink(coords);

      setupSimulationConstructionSet(scs);

      Thread myThread = new Thread(scs);
      myThread.start();

      if (PROFILING)
      {
         System.err.println("TestIMUKalmanFilterSimulationJerryOne::TestIMUKalmanFilterSimulationJerryOne: starting simulation");
         scs.simulate(100.0);
      }
   }


   public void setupSimulationConstructionSet(SimulationConstructionSet simulationConstructionSet)
   {
      simulationConstructionSet.setupGraphGroup("compare filters", new String[][]
      {
         {"estimated_yaw", "estimated_yaw_2", "yaw"}, {"estimated_pitch", "estimated_pitch_2", "pitch"}, {"estimated_roll", "estimated_roll_2", "roll"},
      }, 1);

      simulationConstructionSet.setupGraphGroup("test", new String[][]
      {
         {"x_accel", "y_accel", "z_accel"}, {"x_accel_sensor", "y_accel_sensor", "z_accel_sensor"}, {"x_gyro", "y_gyro", "z_gyro"},
         {"x_gyro_sensor", "y_gyro_sensor", "z_gyro_sensor"},

//       {"x_error", "y_error", "z_error"},
//       {"estimated_roll", "estimated_pitch", "estimated_yaw"},
         {"estimated_roll", "roll", "accel_to_roll"}, {"estimated_pitch", "pitch", "accel_to_pitch"}, {"estimated_yaw", "yaw"}
      }, 2);

      simulationConstructionSet.setupGraphGroup("quaternions", new String[][]
      {
         {"q_qs", "estimated_q0", "estimated_q0_2"}, {"q_qx", "estimated_q1", "estimated_q1_2"}, {"q_qy", "estimated_q2", "estimated_q2_2"},
         {"q_qz", "estimated_q3", "estimated_q3_2"}, {"estimated_qd_wx_bias"}, {"estimated_qd_wy_bias"}, {"estimated_qd_wz_bias"}
      });

      simulationConstructionSet.setupGraphGroup("from accel", new String[][]
      {
         {"q_qs", "q0_from_accel"}, {"q_qx", "q1_from_accel"}, {"q_qy", "q2_from_accel"}, {"q_qz", "q3_from_accel"}, {"yaw", "yaw_from_accel", "yaw_from_quat"},
         {"pitch", "pitch_from_accel", "pitch_from_quat"}, {"roll", "roll_from_accel", "roll_from_quat"},
      });

      simulationConstructionSet.setupEntryBoxGroup("test", new String[]
      {
         "q_noise", "r_noise", "Q_gyro", "R_yaw", "R_roll", "R_pitch", "compass_noise", "accel_noise", "gyro_noise"
      });

      simulationConstructionSet.setupConfiguration("test", "all", "test", "test");
      simulationConstructionSet.setupConfiguration("from accel", "all", "from accel", "test");

//    simulationConstructionSet.selectConfiguration("from accel");
      simulationConstructionSet.selectConfiguration("test");

      MidiSliderBoard evolutionUC33E = new MidiSliderBoard(simulationConstructionSet);

//    evolutionUC33E.setChannel(1, "qd_wx", simulationConstructionSet, -20.0, 20.0, 3.0);
//    evolutionUC33E.setChannel(2, "qd_wy", simulationConstructionSet, -20.0, 20.0, 3.0);
//    evolutionUC33E.setChannel(3, "qd_wz", simulationConstructionSet, -20.0, 20.0, 3.0);

      evolutionUC33E.setSlider(1, "wx_amp", simulationConstructionSet, -20.0, 20.0, 3.0);
      evolutionUC33E.setSlider(2, "wy_amp", simulationConstructionSet, -20.0, 20.0, 3.0);
      evolutionUC33E.setSlider(3, "wz_amp", simulationConstructionSet, -20.0, 20.0, 3.0);

      evolutionUC33E.setSlider(4, "wx_freq", simulationConstructionSet, 0.0, 10.0);
      evolutionUC33E.setSlider(5, "wy_freq", simulationConstructionSet, 0.0, 10.0);
      evolutionUC33E.setSlider(6, "wz_freq", simulationConstructionSet, 0.0, 10.0);

//    evolutionUC33E.setChannel(4, "x_gyro_bias", simulationConstructionSet, -3.0, 3.0, 1.0);
//    evolutionUC33E.setChannel(5, "y_gyro_bias", simulationConstructionSet, -3.0, 3.0, 1.0);
//    evolutionUC33E.setChannel(6, "z_gyro_bias", simulationConstructionSet, -3.0, 3.0, 1.0);


      evolutionUC33E.setSlider(7, "q_noise", simulationConstructionSet, 0.01, 100.0);
      evolutionUC33E.setSlider(8, "r_noise", simulationConstructionSet, 0.01, 100.0);

      simulationConstructionSet.attachExitActionListener(evolutionUC33E);

      simulationConstructionSet.createNewGraphWindow("compare filters", 1, true);
   }

   public static void main(String[] args)
   {
      @SuppressWarnings("unused") TestIMUKalmanFilterSimulationJerryOne testKalmanFilterSimulation = new TestIMUKalmanFilterSimulationJerryOne();
   }

   public DoubleYoVariable[] getControlVars()
   {
      return null;
   }

}
