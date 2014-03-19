package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;


public class AtlasStateEstimatorParameters implements StateEstimatorParameters {
    private final boolean runningOnRealRobot;
	
	public AtlasStateEstimatorParameters(boolean runningOnRealRobot) {
		this.runningOnRealRobot = runningOnRealRobot;
	}

	@Override
	public SensorFilterParameters getSensorFilterParameters(double estimateDT) {
		return new SensorFilterParameters(
	            DRCConfigParameters.JOINT_POSITION_FILTER_FREQ_HZ, DRCConfigParameters.JOINT_VELOCITY_FILTER_FREQ_HZ, 
	            DRCConfigParameters.ORIENTATION_FILTER_FREQ_HZ, DRCConfigParameters.ANGULAR_VELOCITY_FILTER_FREQ_HZ,
	            DRCConfigParameters.LINEAR_ACCELERATION_FILTER_FREQ_HZ, estimateDT);
	}
	
	@Override
	public boolean getAssumePerfectImu() {
		return true;
	}

	@Override
	public boolean getUseSimplePelvisPositionEstimator() {
		return true;
	}

	@Override
	public PointMeasurementNoiseParameters getPointMeasurementNoiseParameters() {
	      return new  PointMeasurementNoiseParameters(
	              DRCConfigParameters.pointVelocityXYMeasurementStandardDeviation,
	              DRCConfigParameters.pointVelocityZMeasurementStandardDeviation,
	              DRCConfigParameters.pointPositionXYMeasurementStandardDeviation,
	              DRCConfigParameters.pointPositionZMeasurementStandardDeviation);		      
	}
//	@Override
//	public boolean getUseStateEstimator() {
//		return true;
//	}
//	@
//	Override
//	public boolean getIntroduceFilteredGaussianPositionError() {
//		return false;
//	}
//	
//	@Override
//	public double getNoiseFilterAlpha() {
//		return 1e-1;
//	}
//
//	@Override
//	public double getPositionNoiseStd() {
//		return 0.01;
//	}
//
//	@Override
//	public double getQuaternionNoiseStd() {
//		return 0.01;
//	}
//


}
