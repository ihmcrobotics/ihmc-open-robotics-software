package us.ihmc.valkyrie.configuration;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.valkyrie.roboNet.api.turbodriver_api.TurbodriverAPI_DRCv4_bench;
import us.ihmc.valkyrie.roboNet.api.turbodriver_api.TurbodriverAPI_DRCv4_ihmc;

public class ValkyriePriorityParameters
{
   public static final PriorityParameters ESTIMATOR_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   public static final PriorityParameters CONTROLLER_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   public static final PriorityParameters IMU_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 10);
   public static final PriorityParameters LOGGER_PRIORITY = new PriorityParameters(45);

   // FQN of the turbodriver we use to test overall round trip time
   public static final Class<?>[] timedTurbodrivers = {TurbodriverAPI_DRCv4_ihmc.class, TurbodriverAPI_DRCv4_bench.class};
   public static final String timingWriteName = "LoopbackWrite";
   public static final String timingReadName = "LoopbackRead";
   
}
