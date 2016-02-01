package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SimulatedMotor {
    	private final double AMBIENT_TEMPERATURE = 25.0;
    
	private final YoVariableRegistry yoVariableRegistry;
	private static int instanceCounter = 0;
	private final double peakMotorTorque;
	private final double peakAmplifierCurrent;
	private final double continuousAmplifierCurrent;
	private final double torqueSensitivity;
	private final double deltaT;

	private final YoVariable motorLimit;
	private final YoVariable amplifierTemperatureState;
	private final YoVariable motorCurrent;
	private final YoVariable availableCurrent;
	private final YoVariable amplifierTemperature;
	private final YoVariable maxAmplifierTemperature;
	private final YoVariable heatTransferRate;
	private final YoVariable effectiveAmplifierResistance;

	

	public SimulatedMotor(YoVariableRegistry parentRegistry, double peakMotorTorque, double peakAmplifierCurrent,
			double continuousAmplifierCurrent, double torqueSensitivity, double deltaT)
	{
		String suffix = "_" + instanceCounter;
		yoVariableRegistry = new YoVariableRegistry("SimMotor" + suffix);
		this.peakAmplifierCurrent = Math.abs(peakAmplifierCurrent);
		this.peakMotorTorque = Math.abs(peakMotorTorque);
		this.continuousAmplifierCurrent = Math.abs(continuousAmplifierCurrent);
		this.torqueSensitivity = Math.abs(torqueSensitivity);
		this.deltaT = deltaT;

		motorLimit = new YoVariable(
				"motorLimit" + suffix,
				"Reason why the output of the motor is not the requested torque",
				MotorLimit.values(), yoVariableRegistry);
		amplifierTemperatureState = new YoVariable("amplifierTemperatureStatus" + suffix, "This is the temperature status of the amplifier",
				AmplifierTemperatureStates.values(), yoVariableRegistry);
		motorCurrent = new YoVariable(
				"motorCurrent" + suffix,
				"Current sent to the motor. Reflects amplifier clipping [Amps]",
				yoVariableRegistry);
		availableCurrent = new YoVariable(
				"availableCurrent" + suffix,
				"This is the instantaneous available current from the amplifier [Amps]",
				yoVariableRegistry);
		amplifierTemperature = new YoVariable("amplifierTemperature" + suffix,
				"This is representative of the amplifier temperature [~C]",
				yoVariableRegistry);
		maxAmplifierTemperature = new YoVariable("maxAmplifierTemperature" + suffix,
				"This is representative of the max allowable amplifier temperature [~C]",
				yoVariableRegistry);
		heatTransferRate = new YoVariable("heatTransferRate" + suffix,
				"This is the rate at which the amplifier cools (0.0 1.0).  1.0 is infinite heat transfer rate while 0.0 is adiabatic",
				yoVariableRegistry);
		effectiveAmplifierResistance = new YoVariable("effectiveAmplifierResistance" + suffix,
				"This is the effective resistance of the motor amplifier as used in ohmic heating (P=i^2*R) [~ohms]",
				yoVariableRegistry);
		
		amplifierTemperature.val = AMBIENT_TEMPERATURE;
		maxAmplifierTemperature.val = 150.0;
		heatTransferRate.val = 0.00008; //0.5;
		effectiveAmplifierResistance.val = 0.065; //0.05;
		
		setAvailableCurrent();
		
		if (parentRegistry != null)
			parentRegistry.addChild(yoVariableRegistry);

		instanceCounter++;
	}

	public double getActualTorqueToMotor(double requestedMotorTorque,
			boolean enforceMotorTorqueLimit) {
		motorLimit.set(MotorLimit.NONE);

		double ret = requestedMotorTorque;
		
		double requestedCurrent = ret / torqueSensitivity;
		setAvailableCurrent();
		
		if (enforceMotorTorqueLimit)
		{
		    if (Math.abs(requestedCurrent) > availableCurrent.val) {
			ret = Math.signum(ret) * availableCurrent.val * torqueSensitivity;
			motorLimit.set(MotorLimit.AMPLIFIER);
		    }

		    if (Math.abs(ret) > peakMotorTorque) {
			ret = Math.signum(ret) * peakMotorTorque;
			motorLimit.set(MotorLimit.PEAK_TORQUE);
		    }
		}
		

		motorCurrent.val = ret / torqueSensitivity;

		return ret;
	}
	
	private void setAvailableCurrent()
	{
		amplifierTemperature.val = effectiveAmplifierResistance.val*motorCurrent.val*motorCurrent.val*deltaT + (1.0 - heatTransferRate.val)*amplifierTemperature.val;
		
		if (amplifierTemperature.val < AMBIENT_TEMPERATURE)
		    amplifierTemperature.val = AMBIENT_TEMPERATURE;
		
		if (amplifierTemperature.val > maxAmplifierTemperature.val)
		{
			availableCurrent.val = continuousAmplifierCurrent;
			amplifierTemperatureState.set(AmplifierTemperatureStates.HOT);
		}
		else
		{
			availableCurrent.val = peakAmplifierCurrent;
			amplifierTemperatureState.set(AmplifierTemperatureStates.COOL);
		}
	}

	private enum MotorLimit {
		NONE, AMPLIFIER, PEAK_TORQUE
	};
	
	private enum AmplifierTemperatureStates {
		COOL, HOT
	};
}
