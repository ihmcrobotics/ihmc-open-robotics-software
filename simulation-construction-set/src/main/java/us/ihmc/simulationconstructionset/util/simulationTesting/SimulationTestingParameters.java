package us.ihmc.simulationconstructionset.util.simulationTesting;

import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SimulationTestingParameters extends SimulationConstructionSetParameters
{
   public static final String RUN_MULTI_THREADED = "run.multi.threaded";
   public static final String USE_PERFECT_SENSORS = "use.perfect.sensors";
   public static final String CREATE_SCS_VIDEOS = "create.scs.videos";
   public static final String CHECK_NOTHING_CHANGED_IN_SIMULATION = "check.nothing.changed.in.simulation";
   public static final String KEEP_SCS_UP = "keep.scs.up";

   public SimulationTestingParameters()
   {
      super();

      parameters.put(RUN_MULTI_THREADED, new BooleanHolder("true"));
      parameters.put(USE_PERFECT_SENSORS, new BooleanHolder("false"));
      parameters.put(CREATE_SCS_VIDEOS, new BooleanHolder("false"));
      parameters.put(CHECK_NOTHING_CHANGED_IN_SIMULATION, new BooleanHolder("false"));
      parameters.put(KEEP_SCS_UP, new BooleanHolder("false"));
   }

   public static SimulationTestingParameters createFromSystemProperties()
   {
      SimulationTestingParameters parameters = new SimulationTestingParameters();
      parameters.setFromSystemProperties();
      return parameters;
   }

   /**
    * @deprecated Use {@link #createFromSystemProperties()} instead.
    */
   public static SimulationTestingParameters createFromEnvironmentVariables()
   {
      return createFromSystemProperties();
   }

   public boolean getUsePefectSensors()
   {
      return ((BooleanHolder) parameters.get(USE_PERFECT_SENSORS)).value;
   }

   public void setUsePefectSensors(boolean value)
   {
      ((BooleanHolder) parameters.get(USE_PERFECT_SENSORS)).value = value;
   }

   public void setRunMultiThreaded(boolean value)
   {
      ((BooleanHolder) parameters.get(RUN_MULTI_THREADED)).value = value;
   }

   public boolean getRunMultiThreaded()
   {
      return ((BooleanHolder) parameters.get(RUN_MULTI_THREADED)).value;
   }

   public boolean getCreateSCSVideos()
   {
      return ((BooleanHolder) parameters.get(CREATE_SCS_VIDEOS)).value;
   }

   public void setCreateSCSVideos(boolean value)
   {
      ((BooleanHolder) parameters.get(CREATE_SCS_VIDEOS)).value = value;
   }

   public boolean getCheckNothingChangedInSimulation()
   {
      return ((BooleanHolder) parameters.get(CHECK_NOTHING_CHANGED_IN_SIMULATION)).value;
   }

   public void setCheckNothingChangedInSimulation(boolean value)
   {
      ((BooleanHolder) parameters.get(CHECK_NOTHING_CHANGED_IN_SIMULATION)).value = value;
   }

   public boolean getKeepSCSUp()
   {
      return ((BooleanHolder) parameters.get(KEEP_SCS_UP)).value;
   }

   public void setKeepSCSUp(boolean value)
   {
      ((BooleanHolder) parameters.get(KEEP_SCS_UP)).value = value;
   }
}
