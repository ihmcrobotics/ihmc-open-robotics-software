package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

public interface WholeBodyControllerParameters extends FullHumanoidRobotModelFactory
{
	public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters();

	public ICPOptimizationParameters getICPOptimizationParameters();

	public WalkingControllerParameters getWalkingControllerParameters();

	public RobotContactPointParameters getContactPointParameters();

	public double getControllerDT();

   public DRCRobotSensorInformation getSensorInformation();

}
