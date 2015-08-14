package us.ihmc.moonwalking.models.SeriesElasticActuator;

//~--- non-JDK imports --------------------------------------------------------

import com.yobotics.simulationconstructionset.RobotController;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.YoVariableType;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class DesiredTorqueController implements RobotController
{
    private static int instanceCounter = 0;
    private final String suffix = "_" + instanceCounter;
    private final YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("DesiredController" + suffix);
    private final YoVariable positionErrorDot = new YoVariable("positionErrorDot" + suffix,
                                                    "Error in desried velocity versus actual output velocity. Units are rad/sec", yoVariableRegistry);
    private final YoVariable positionError = new YoVariable("positionError" + suffix, "Error in desried position versus actual output postion. Units are rad",
                                                 yoVariableRegistry);
    private final YoVariable outputVelActual = new YoVariable("outputVelActual" + suffix, "Actual output velocity. Units are rad/s", yoVariableRegistry);
    private final YoVariable outputPosDesired = new YoVariable("outputPosDesired" + suffix, "Desired output postion. Units are rad", yoVariableRegistry);
    private final YoVariable outputPosActual = new YoVariable("outputPosActual" + suffix, "Actual output postion. Units are rad", yoVariableRegistry);
    private final YoVariable kpPosition = new YoVariable("kpPosition" + suffix, "Proportional control gain for actuator output position control. Units are Nm/rad",
                                              yoVariableRegistry);
    private final YoVariable kdPosition = new YoVariable("kdPosition" + suffix, "Damping control gain for actuator output position control. Units are Nm/(rad/sec)",
                                              yoVariableRegistry);
    private final YoFunctionGenerator functionGeneratorTorque = new YoFunctionGenerator("generatorTau" + suffix, yoVariableRegistry);
    private final YoFunctionGenerator functionGeneratorPosition = new YoFunctionGenerator("generatorPos" + suffix, yoVariableRegistry);
    private final YoVariable desiredTorqueForController = new YoVariable("desiredTorqueForController" + suffix, yoVariableRegistry);
    private final YoVariable controllerModel = new YoVariable("controllerModel" + suffix,
                                                   "This sets the mode (and desired torque) for the SEA controller: position, force, etc.",
                                                   ControllerMode.values(), yoVariableRegistry);
    private final ActuatorController actuatorController;
    private final SeriesElasticActuatorModel seriesElasticActuatorModel;
    private final YoVariable time;
    private final YoVariable callCounterDTC =  new YoVariable("callCounterDTC" + suffix, YoVariableType.INT, yoVariableRegistry);

    
    public enum ControllerMode {TORQUE, POSITION, OPEN_LOOP_TAU}
    public enum TestToConduct {TORQUE_CHIRP, TORQUE_SQUARE, OPEN_LOOP_TORQUE, OPEN_LOOP_TORQUE_CHIRP,
	POSITION_CHIRP, POSITION_SQUARE, POSITION_SINE}

    public static TestToConduct testToConduct;

    public DesiredTorqueController(ActuatorController actuatorController, SeriesElasticActuatorModel seriesElasticActuatorModel)
    {
        this.actuatorController = actuatorController;
        this.seriesElasticActuatorModel = seriesElasticActuatorModel;
        this.time = seriesElasticActuatorModel.t;

        kpPosition.val = 2000.0;
        kdPosition.val = 90.0;

        switch(testToConduct)
        {
        case TORQUE_CHIRP:
	{
	    setupForTorqueChirp();
	    break;
	}
        case TORQUE_SQUARE:
	{
	    setupForTorqueSquareWave();
	    break;
	}
        case OPEN_LOOP_TORQUE:
        {
            setupForOpenLoopTorque();
            break;
        }
        case OPEN_LOOP_TORQUE_CHIRP:
        {
            throw new RuntimeException("method not implemented yet");
        }
        case POSITION_CHIRP:
	{
	    setupForPositionChirp();
	    break;
	}
        case POSITION_SQUARE:
	{
	    setupForPositionSquareWave();
	    break;
	}
        case POSITION_SINE:
        {
            setupForPositionSineWave();
            break;
        }
        }
          
        instanceCounter++;
    }

    private void setupForOpenLoopTorque()
    {
        controllerModel.set(ControllerMode.OPEN_LOOP_TAU);
    }

    private void setupForPositionChirp()
    {
        functionGeneratorPosition.setPauseTime(0.05);
        functionGeneratorPosition.setAmplitude(SeriesElasticActuatorSimulation.PEAK_POSITION_TO_TRACK);    // 10.0);
        functionGeneratorPosition.setChirpRate(2.0);    // 40.0);
        functionGeneratorPosition.setResetTime(20.0);
        functionGeneratorPosition.setMode(YoFunctionGeneratorMode.CHIRP);


        controllerModel.set(ControllerMode.POSITION);
    }

    private void setupForPositionSineWave()
    {
        functionGeneratorPosition.setAmplitude(SeriesElasticActuatorSimulation.PEAK_POSITION_TO_TRACK);
        functionGeneratorPosition.setFrequency(0.2);
        functionGeneratorPosition.setMode(YoFunctionGeneratorMode.SINE);

        controllerModel.set(ControllerMode.POSITION);
    }

    private void setupForPositionSquareWave()
    {
        functionGeneratorPosition.setAmplitude(SeriesElasticActuatorSimulation.PEAK_POSITION_TO_TRACK);
        functionGeneratorPosition.setFrequency(1.0);
        functionGeneratorPosition.setMode(YoFunctionGeneratorMode.SQUARE);

        controllerModel.set(ControllerMode.POSITION);
    }

    private void setupForTorqueChirp()
    {
        functionGeneratorTorque.setPauseTime(0.05);
        functionGeneratorTorque.setAmplitude(SeriesElasticActuatorSimulation.PEAK_TORQUE_TO_TRACK);    // 10.0);
        functionGeneratorTorque.setChirpRate(5.0);     // 40.0);
        functionGeneratorTorque.setResetTime(20.0);

        functionGeneratorTorque.setMode(YoFunctionGeneratorMode.CHIRP);

        controllerModel.set(ControllerMode.TORQUE);
    }

    private void setupForTorqueSquareWave()
    {
        functionGeneratorTorque.setAmplitude(SeriesElasticActuatorSimulation.PEAK_TORQUE_TO_TRACK);
        functionGeneratorTorque.setFrequency(0.5);
        functionGeneratorTorque.setMode(YoFunctionGeneratorMode.SQUARE);

        controllerModel.set(ControllerMode.TORQUE);
    }

    public void doControl()
    {
	callCounterDTC.val++;
	
        double desiredTorque;

        outputPosActual.val = seriesElasticActuatorModel.getActuatorOutputPosition();
        outputVelActual.val = seriesElasticActuatorModel.getActuatorOutputVelocity();

        switch ((ControllerMode) controllerModel.getEnumValue())
        {
            case TORQUE :
            {
                desiredTorque = functionGeneratorTorque.getValue(time.val);

                break;
            }

            case POSITION :
            {
                outputPosDesired.val = functionGeneratorPosition.getValue(time.val);

                positionError.val = outputPosDesired.val - outputPosActual.val;
                positionErrorDot.val = 0.0 - outputVelActual.val;

                desiredTorque = kpPosition.val * positionError.val + kdPosition.val * positionErrorDot.val;

                break;
            }

            case OPEN_LOOP_TAU :
            {
                actuatorController.setTorqueFeedbackGains(0.0, 0.0);
                desiredTorque = this.desiredTorqueForController.val;

                break;
            }

            default :
            {
                throw new RuntimeException("not a valid value: controllerModel.getEnumValue()=" + controllerModel.getEnumValue());
            }
        }

        actuatorController.setDesiredTorque(desiredTorque);
    }

    public YoVariableRegistry getYoVariableRegistry()
    {
        return yoVariableRegistry;
    }

    ;
}
