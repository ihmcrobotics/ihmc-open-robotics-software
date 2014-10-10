package us.ihmc.commonWalkingControlModules.configurations;

public interface CapturePointPlannerParameters
{
	public abstract double getDoubleSupportInitialTransferDuration();
	
	public abstract double getDoubleSupportDuration();
	
	public abstract double getSingleSupportDuration();
	
	public abstract int getNumberOfFootstepsToConsider();
	
	public abstract int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory();
	
	public abstract int getNumberOfFootstepsToStop();
	
	public abstract double getIsDoneTimeThreshold();
}
