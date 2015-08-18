package us.ihmc.wholeBodyController;
 
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public interface WholeBodyControllerParameters extends SDFFullRobotModelFactory
{
	public CapturePointPlannerParameters getCapturePointPlannerParameters();

	public ArmControllerParameters getArmControllerParameters();

	public WalkingControllerParameters getWalkingControllerParameters();
	
	public WalkingControllerParameters getMultiContactControllerParameters();
	
	public DRCRobotContactPointParameters getContactPointParameters();
	
	public double getControllerDT();

	public SDFHumanoidRobot createSdfRobot(boolean createCollisionMeshes);
	
	public abstract GeneralizedSDFRobotModel getGeneralizedRobotModel();
	
	public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel);
	
	public abstract WholeBodyIkSolver createWholeBodyIkSolver();
	
	public DefaultArmConfigurations getDefaultArmConfigurations();
}
