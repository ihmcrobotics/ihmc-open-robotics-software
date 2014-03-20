package us.ihmc.valkyrie.configuration;

import us.ihmc.realtime.PriorityParameters;

public class ValkyrieTimingParameters
{
   public static final PriorityParameters ESTIMATOR_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   public static final PriorityParameters CONTROLLER_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   public static final PriorityParameters IMU_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 10);
   
   public static final long ESTIMATOR_DT_IN_NS = 2000000;
   public static final long CONTROL_DT_IN_NS = 6000000;
   

   // FQN of the turbodriver we use to test overall round trip time
   public static final String timingNode = "/pelvis/wj1";
   public static final String timingWriteName = "LoopbackWrite";
   public static final String timingReadName = "LoopbackRead";
   
}
