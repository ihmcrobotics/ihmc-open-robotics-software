package us.ihmc.kalman.comparisons;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class CompareYoKalmanSetups
{
   
   private static final boolean IS_NON_REAL_TIME_WITH_VARIABLE_DT = true;
   private final SimulationConstructionSet sim;
   private final Robot emptyRobot;
   private final ImuAsInputKalmanSetup imuAsInputKalman;
   private final ConstantAccelerationKalmanSetup piecewiseConstantAccelerationKalman;
   private final SinosoidalExampleFunction function;
   private final NoisifierAndDelayer noisifier;
   private static final double EXAMPLE_PHASE = Math.PI / 4.0;
   private static final double EXAMPLE_FREQUENCY = 0.01;
   private static final double EXAMPLE_AMPLITUDE = 4.0;
   
   private static final double SCS_DT = 0.0001;
   private static final int SIM_TICKS_PER_CONTROL_TICK = 100;
   private static final int RECORD_FREQUENCY = 10;
   
   public CompareYoKalmanSetups()
   {
     //start scs to plot
      emptyRobot = new Robot("emptyRobot");
      sim = new SimulationConstructionSet(emptyRobot);
      sim.setDT(SCS_DT, RECORD_FREQUENCY);
      sim.setGroundVisible(false);
      
      function = new SinosoidalExampleFunction(emptyRobot, EXAMPLE_AMPLITUDE, EXAMPLE_FREQUENCY, EXAMPLE_PHASE);
      emptyRobot.setController(function);
      
      noisifier = new NoisifierAndDelayer(emptyRobot, function);
      emptyRobot.setController(noisifier, SIM_TICKS_PER_CONTROL_TICK);
      
      imuAsInputKalman = new ImuAsInputKalmanSetup(noisifier, SIM_TICKS_PER_CONTROL_TICK * SCS_DT, IS_NON_REAL_TIME_WITH_VARIABLE_DT);
      emptyRobot.setController(imuAsInputKalman, SIM_TICKS_PER_CONTROL_TICK);
      
      piecewiseConstantAccelerationKalman = new ConstantAccelerationKalmanSetup(noisifier, SIM_TICKS_PER_CONTROL_TICK * SCS_DT, IS_NON_REAL_TIME_WITH_VARIABLE_DT);
      emptyRobot.setController(piecewiseConstantAccelerationKalman, SIM_TICKS_PER_CONTROL_TICK);
            
      sim.startOnAThread();
    }

   public static void main(String[] args)
   {
      new CompareYoKalmanSetups();
   }

}
