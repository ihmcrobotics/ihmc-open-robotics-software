package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.RobotController;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class CurrentLimitSimulation {

	public CurrentLimitSimulation()
	{
		Robot robot = new Robot("robot");
				
		double peakMotorTorque = 1e9;
		double peakAmplifierCurrent = 30.0;
		double continuousAmplifierCurrent = 15.0;
		double torqueSensitivity = 1.0;
		double deltaT = 0.001;
				
		YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("motor");
		
		SimulatedMotor motor = new SimulatedMotor(yoVariableRegistry, peakMotorTorque,
				peakAmplifierCurrent, continuousAmplifierCurrent, torqueSensitivity, deltaT);
		
		robot.addYoVariableRegistry(yoVariableRegistry);
		
		CurrentLimitController controller = new CurrentLimitController(motor);
		robot.setController(controller, 1);
		
		SimulationConstructionSet scs = new SimulationConstructionSet(robot);
		scs.setDT(deltaT, 1);
		scs.changeBufferSize(32000);
		
		Thread simThread = new Thread(scs);
		setUpGUI(scs);
		simThread.start();
		scs.hideViewport();
	}
	
	private void setUpGUI(SimulationConstructionSet scs)
    {
		scs.setupEntryBoxGroup("Control", new String[]{ "requestedMotorTorque", "heatTransferRate_0", "effectiveAmplifierResistance_0"});
		
		
		scs.setupGraphGroup("Control", new String[][][]
		                                           	{
		                                           	{{ "amplifierTemperature_0"},{ "auto" }},
		                                           	{{ "amplifierTemperatureStatus_0"},{ "" }}, 
		                                           	//{{"actualMotorTorque"},{"auto"}},
		                                           	{{"t"},{""}},
		                                           	{{"availableCurrent_0", "motorCurrent_0"},{"auto"}},
		                                           	}, 1);
		
		scs.setupConfiguration("Control", "all", "Control", "Control");
		scs.selectConfiguration("Control");
    }
	
	public static void main(String[] args) {
		CurrentLimitSimulation currentLimitSimulation = new CurrentLimitSimulation();
	}
	
	private class CurrentLimitController implements RobotController 
	{
		private final YoVariableRegistry yoVariableRegistry;
		
		private final SimulatedMotor motor;
		
		private final YoVariable requestedMotorTorque;
		private final YoVariable actualMotorTorque;
		
		public CurrentLimitController(SimulatedMotor motor)
		{
			yoVariableRegistry = new YoVariableRegistry("controller");
			
			this.motor = motor;
			
			requestedMotorTorque = new YoVariable("requestedMotorTorque", yoVariableRegistry);
			actualMotorTorque = new YoVariable("actualMotorTorque", yoVariableRegistry);
		}
		
		public void doControl() {
			actualMotorTorque.val = motor.getActualTorqueToMotor(requestedMotorTorque.val, true);	
		}

		public YoVariableRegistry getYoVariableRegistry() {
			// TODO Auto-generated method stub
			return yoVariableRegistry;
		}
		
	}
}

