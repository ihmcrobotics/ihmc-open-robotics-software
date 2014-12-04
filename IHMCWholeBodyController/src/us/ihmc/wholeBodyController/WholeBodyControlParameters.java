package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public interface WholeBodyControlParameters
{
	public abstract CapturePointPlannerParameters getCapturePointPlannerParameters();

	public abstract ArmControllerParameters getArmControllerParameters();

	public abstract WalkingControllerParameters getWalkingControllerParameters();
	
	public abstract WalkingControllerParameters getMultiContactControllerParameters();
	
	public abstract DRCRobotContactPointParameters getContactPointParameters();
	
	public abstract double getControllerDT();
}
