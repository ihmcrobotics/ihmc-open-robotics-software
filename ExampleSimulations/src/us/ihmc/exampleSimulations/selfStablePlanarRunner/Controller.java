package us.ihmc.exampleSimulations.selfStablePlanarRunner;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;

public class Controller implements RobotController {

    private YoDouble q_wheel, qd_wheel, qdd_wheel, tau_wheel;
    private YoDouble q_l_knee, qd_l_knee, qdd_l_knee, tau_l_knee;
    private YoDouble q_r_knee, qd_r_knee, qdd_r_knee, tau_r_knee;
    private final YoVariableRegistry registry = new YoVariableRegistry("Controller");
    private final YoDouble b_wheel = new YoDouble("b_wheel", registry);
    private final YoDouble qd_d_wheel = new YoDouble("qd_d_wheel", registry);
    private final YoDouble k_knee = new YoDouble("k_knee", registry);
    private final YoDouble b_knee = new YoDouble("b_knee", registry);
    
    private final SelfStablePlanarRunner_Robot robot;
    
    public Controller (SelfStablePlanarRunner_Robot robot) {
	this.robot = robot;
	
	initControl();
    }
    
    private void initControl() {
	q_wheel   = (YoDouble) robot.getVariable(  "q_driving_wheel");
	qd_wheel  = (YoDouble) robot.getVariable( "qd_driving_wheel");
	qdd_wheel = (YoDouble) robot.getVariable("qdd_driving_wheel");
	tau_wheel = (YoDouble) robot.getVariable("tau_driving_wheel");
	q_l_knee   = (YoDouble) robot.getVariable(  "q_l_knee");
	qd_l_knee  = (YoDouble) robot.getVariable( "qd_l_knee");
	qdd_l_knee = (YoDouble) robot.getVariable("qdd_l_knee");
	tau_l_knee = (YoDouble) robot.getVariable("tau_l_knee");
	q_r_knee   = (YoDouble) robot.getVariable(  "q_r_knee");
	qd_r_knee  = (YoDouble) robot.getVariable( "qd_r_knee");
	qdd_r_knee = (YoDouble) robot.getVariable("qdd_r_knee");
	tau_r_knee = (YoDouble) robot.getVariable("tau_r_knee");
	
	qd_d_wheel.set(6.0);
	b_wheel.set(100.0);
	if        (robot.getName() == "SelfStableRunner1") {
		k_knee.set(296.51);
	} else if (robot.getName() == "SelfStableRunner2") {
		k_knee.set(275);
	} else if (robot.getName() == "SelfStableRunner3") {
		k_knee.set(322.1);
	} else if (robot.getName() == "SelfStableRunner4") {
		k_knee.set(368.57);
	} else if (robot.getName() == "SelfStableRunner5") {
		k_knee.set(404.8);
	} else if (robot.getName() == "SelfStableRunner6") {
		k_knee.set(440.88);
	} else if (robot.getName() == "SelfStableRunner7") {
		k_knee.set(478.07);
	} else {
		k_knee.set(400);
	}
	b_knee.set(1);
    }
    
    public void initialize() {
    }

    public YoVariableRegistry getYoVariableRegistry() {
	return registry;
    }

    public String getName() {
	return null;
    }

    public String getDescription() {
	return null;
    }

    public void doControl() {
	tau_wheel.set(b_wheel.getDoubleValue() * (qd_d_wheel.getDoubleValue() - qd_wheel.getDoubleValue()));

	tau_l_knee.set(- k_knee.getDoubleValue() * q_l_knee.getDoubleValue() - b_knee.getDoubleValue() * qd_l_knee.getDoubleValue());
	tau_r_knee.set(- k_knee.getDoubleValue() * q_r_knee.getDoubleValue() - b_knee.getDoubleValue() * qd_r_knee.getDoubleValue());

    }

}
