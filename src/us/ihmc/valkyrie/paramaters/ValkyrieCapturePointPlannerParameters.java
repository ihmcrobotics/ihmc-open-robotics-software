package us.ihmc.valkyrie.paramaters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

public class ValkyrieCapturePointPlannerParameters implements CapturePointPlannerParameters
{
	private boolean runningOnRealRobot;

	public ValkyrieCapturePointPlannerParameters(boolean runningOnRealRobot)
	{
		this.runningOnRealRobot = runningOnRealRobot;
	}
	
	@Override
	public double getDoubleSupportInitialTransferDuration()
	{
		return runningOnRealRobot ? 2.0 : 1.0; 
	}

	@Override
	public double getDoubleSupportDuration()
	{
		return runningOnRealRobot ? 1.5 : 0.25;
	}

	@Override
	public double getSingleSupportDuration()
	{
		return runningOnRealRobot ? 1.5 : 0.7;
	}

	@Override
	public int getNumberOfFootstepsToConsider()
	{
		return 3;
	}

	@Override
	public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
	{
		return 5;
	}

	@Override
	public int getNumberOfFootstepsToStop()
	{
		return 2;
	}
}
