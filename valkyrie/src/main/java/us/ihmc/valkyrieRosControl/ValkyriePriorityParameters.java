package us.ihmc.valkyrieRosControl;

import us.ihmc.realtime.PriorityParameters;

public class ValkyriePriorityParameters
{
   public static final PriorityParameters ESTIMATOR_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   public static final PriorityParameters CONTROLLER_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   public static final PriorityParameters IMU_PRIORITY = new PriorityParameters(PriorityParameters.getMaximumPriority() - 10);
   public static final PriorityParameters LOGGER_PRIORITY = new PriorityParameters(40);
   public static final PriorityParameters HAND_COMMUNICATOR_PRIORITY = new PriorityParameters(35);
   public static final PriorityParameters POSECOMMUNICATOR_PRIORITY = new PriorityParameters(45);
}
