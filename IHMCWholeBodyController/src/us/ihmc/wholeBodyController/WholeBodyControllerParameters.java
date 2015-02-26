package us.ihmc.wholeBodyController;
 
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModelFactory;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public interface WholeBodyControllerParameters extends FullRobotModelFactory
{
	public CapturePointPlannerParameters getCapturePointPlannerParameters();

	public ArmControllerParameters getArmControllerParameters();

	public WalkingControllerParameters getWalkingControllerParameters();
	
	public WalkingControllerParameters getMultiContactControllerParameters();
	
	public DRCRobotContactPointParameters getContactPointParameters();
	
	public double getControllerDT();
	
	public SDFFullRobotModel createFullRobotModel();

	public SDFRobot createSdfRobot(boolean createCollisionMeshes);
	
	public abstract GeneralizedSDFRobotModel getGeneralizedRobotModel();
	
	public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel);
	
	public abstract WholeBodyIkSolver createWholeBodyIkSolver();
	
	public DefaultArmConfigurations getDefaultArmConfigurations();
}
