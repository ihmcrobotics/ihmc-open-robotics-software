package us.ihmc.wholeBodyController;
 
import us.ihmc.humanoidRobotics.HumanoidFloatingRootJointRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public interface WholeBodyControllerParameters extends FullHumanoidRobotModelFactory
{
	public CapturePointPlannerParameters getCapturePointPlannerParameters();

	public ICPOptimizationParameters getICPOptimizationParameters();

	public ArmControllerParameters getArmControllerParameters();

	public WalkingControllerParameters getWalkingControllerParameters();
	
	public WalkingControllerParameters getMultiContactControllerParameters();
	
	public RobotContactPointParameters getContactPointParameters();
	
	public double getControllerDT();

	public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes);
	
	
	public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel);
	
	public DefaultArmConfigurations getDefaultArmConfigurations();
}
