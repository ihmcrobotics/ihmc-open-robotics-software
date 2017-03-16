package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public interface WholeBodyControllerParameters extends FullHumanoidRobotModelFactory
{
	public CapturePointPlannerParameters getCapturePointPlannerParameters();

	public ICPOptimizationParameters getICPOptimizationParameters();

	public WalkingControllerParameters getWalkingControllerParameters();

	public RobotContactPointParameters getContactPointParameters();

	public double getControllerDT();

	public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel);

	public DefaultArmConfigurations getDefaultArmConfigurations();
}
