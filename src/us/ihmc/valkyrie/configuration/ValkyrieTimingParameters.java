package us.ihmc.valkyrie.configuration;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.valkyrie.roboNet.api.turbodriver_api.TurbodriverAPI_DRCv4_bench;
import us.ihmc.valkyrie.roboNet.api.turbodriver_api.TurbodriverAPI_DRCv4_ihmc;

public class ValkyrieTimingParameters
{
   public static final PriorityParameters ESTIMATOR_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   public static final PriorityParameters CONTROLLER_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   public static final PriorityParameters IMU_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 10);
   
   public static final long ESTIMATOR_DT_IN_NS = 2000000;
   public static final long CONTROL_DT_IN_NS = 6000000;
   

   // FQN of the turbodriver we use to test overall round trip time
   public static final Class<?>[] timedTurbodrivers = {TurbodriverAPI_DRCv4_ihmc.class, TurbodriverAPI_DRCv4_bench.class};
   public static final String timingWriteName = "LoopbackWrite";
   public static final String timingReadName = "LoopbackRead";
   
}
