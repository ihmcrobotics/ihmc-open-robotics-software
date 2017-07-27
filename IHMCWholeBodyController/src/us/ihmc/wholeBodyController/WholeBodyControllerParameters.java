package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

public interface WholeBodyControllerParameters
{
   public double getControllerDT();

   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters();

	public ICPOptimizationParameters getICPOptimizationParameters();

	public WalkingControllerParameters getWalkingControllerParameters();

	public RobotContactPointParameters getContactPointParameters();

   public DRCRobotSensorInformation getSensorInformation();
}
