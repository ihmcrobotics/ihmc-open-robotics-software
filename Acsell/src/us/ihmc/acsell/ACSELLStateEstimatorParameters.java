package us.ihmc.acsell;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;

public class ACSELLStateEstimatorParameters implements StateEstimatorParameters {
	private final boolean runningOnRealRobot;

	public ACSELLStateEstimatorParameters(boolean runningOnRealRobot) {
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

}
