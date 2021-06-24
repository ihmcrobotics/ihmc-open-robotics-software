package us.ihmc.valkyrieRosControl;

import us.ihmc.realtime.PriorityParameters;

public class ValkyriePriorityParameters
{
   public static final PriorityParameters ESTIMATOR_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   public static final PriorityParameters CONTROLLER_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   public static final PriorityParameters IMU_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 10);
   public static final PriorityParameters POSECOMMUNICATOR_PRIORITY = new PriorityParameters(45);
   public static final PriorityParameters JVM_STATISTICS_PRIORITY = new PriorityParameters(10);
}
