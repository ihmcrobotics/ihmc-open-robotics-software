package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepPlanningParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

public interface WholeBodyControllerParameters
{
   public double getControllerDT();

   /**
    * Returns the parameters used to create Footstep Plans.
    */
   default public FootstepPlanningParameters getFootstepPlanningParameters()
   {
      return null;
   }

   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters();

	public ICPOptimizationParameters getICPOptimizationParameters();

	public WalkingControllerParameters getWalkingControllerParameters();

	public RobotContactPointParameters getContactPointParameters();

   public DRCRobotSensorInformation getSensorInformation();
}
