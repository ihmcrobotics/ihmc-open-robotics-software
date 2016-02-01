package us.ihmc.moonwalking.models.SeriesElasticActuator;

//~--- non-JDK imports --------------------------------------------------------

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.LinkGraphics;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.YoVariableType;
import com.yobotics.simulationconstructionset.util.graphics.*;

//~--- JDK imports ------------------------------------------------------------

import java.util.ArrayList;

import javax.vecmath.Vector3d;

/**
 * <p>
 * Title:
 * </p>
 * 
 * <p>
 * Description:
 * </p>
 * 
 * <p>
 * Copyright: Copyright (c) 2009
 * </p>
 * 
 * <p>
 * Company:
 * </p>
 * 
 * @author not attributable
 * @version 1.0
 */
public class SeriesElasticActuatorModel extends Robot
{
    public static final boolean SHOW_COORDINATE_SYSTEM = true;
    private static int instanceCounter = 0;
    private ArrayList<DynamicGraphicObjectsList> dynamicGraphicObjectsList = new ArrayList<DynamicGraphicObjectsList>();
    private final PinJoint actuatorOutputShaft;
    private final PinJoint gearboxOutputShaft;
    private final PinJoint motorShaftJoint;
    public final YoVariable t;

    private final SimulatedEncoder gearboxOutputEncoder;
    private final SimulatedEncoder actuatorOutputEncoder;
    
    private final EncoderStateMachineVelocityEstimator gearboxOutputEncoderStateMachineVelocityEstimator;
    private final EncoderStateMachineVelocityEstimator actuatorOutputEncoderStateMachineVelocityEstimator;

    private final YoVariable gearboxOutputEncoderTicks;
    private final YoVariable actuatorOutputRealtiveToGroundEncoderTicks;

    private final YoVariable gearboxOutputPositionFromEncoder;
    private final YoVariable actuatorOutputPositionFromEncoder;
    private final YoVariable gearboxOutputVelocityFromEncoder;
    private final YoVariable actuatorOutputVelocityFromEncoder;
    
    public SeriesElasticActuatorModel(double encoderResolutionCountsPerDistance)
    {
	super("RSEA" + "_" + instanceCounter);

	String suffix = "_" + instanceCounter;

	this.t = getVariable("t");

	double xOffset = 5.0 * instanceCounter * SEAParameters.MOTOR_BODY_RADIUS;

	Link motorBaseLink = getMotorBaseLink(xOffset);
	this.addStaticLink(motorBaseLink);

	gearboxOutputEncoder = new SimulatedEncoder(encoderResolutionCountsPerDistance, this, "gear");
	actuatorOutputEncoder = new SimulatedEncoder(encoderResolutionCountsPerDistance, this, "actuator");

	YoVariableRegistry registry = new YoVariableRegistry("Robot Encoders" + suffix);
	
	gearboxOutputEncoderTicks = new YoVariable("gearboxTicks" + suffix, "Ticks from encoder of gearbox output position relative to ground [ticks]", YoVariableType.INT, registry);
	actuatorOutputRealtiveToGroundEncoderTicks = new YoVariable("actuatorTicks" + suffix, "Ticks from encoder of output position relative to ground [tciks]", YoVariableType.INT, registry);
	gearboxOutputPositionFromEncoder = new YoVariable("gearboxOutputPositionFromEncoder" + suffix, "Gearbox output position as measured by the encoder [rad]", registry);
	actuatorOutputPositionFromEncoder = new YoVariable("actuatorOutputPositionFromEncoder" + suffix,  "Actuator output position as measured by the encoder [rad]", registry);
	gearboxOutputVelocityFromEncoder = new YoVariable("gearboxOutputVelocityFromEncoder" + suffix,  "Gearbox output velocity as measured by the encoder [rad/s]", registry);
	actuatorOutputVelocityFromEncoder = new YoVariable("actuatorOutputVelocityFromEncoder" + suffix,  "Actutaor output velocity as measured by the encoder [rad/s]", registry);
	
	
	gearboxOutputEncoderStateMachineVelocityEstimator = new EncoderStateMachineVelocityEstimatorTwo("gearbox", gearboxOutputEncoderTicks, this.t, this);
	actuatorOutputEncoderStateMachineVelocityEstimator = new EncoderStateMachineVelocityEstimatorTwo("actuator", actuatorOutputRealtiveToGroundEncoderTicks, this.t, this);
	
//	gearboxOutputEncoderStateMachineVelocityEstimator = new EncoderStateMachineVelocityEstimatorSimple("gearbox", gearboxOutputEncoderTicks, this.t, this);
//	actuatorOutputEncoderStateMachineVelocityEstimator = new EncoderStateMachineVelocityEstimatorSimple("actuator", actuatorOutputRealtiveToGroundEncoderTicks, this.t, this);
	
	this.addYoVariableRegistry(registry);


	Vector3d shaftOffset = new Vector3d(xOffset, 0.0, SEAParameters.MOTOR_BODY_RADIUS);
	motorShaftJoint = new PinJoint("motorShaft" + suffix, shaftOffset, this, Joint.Z);

	// motorShaftJoint.setDamping(0.001);

	this.addRootJoint(motorShaftJoint);

	Link motorShaftLink = getMotorShaftLink();
	motorShaftJoint.setLink(motorShaftLink);

	Vector3d gearboxOutputShaftOffset = new Vector3d(xOffset, 0.0, 3.0 * SEAParameters.MOTOR_BODY_RADIUS);
	gearboxOutputShaft = new PinJoint("gearboxOutputShaft" + suffix, gearboxOutputShaftOffset, this, Joint.Z);

	this.addRootJoint(gearboxOutputShaft);
	Link gearboxOutputShaftLink = getGearboxShaftLink();

	gearboxOutputShaft.setLink(gearboxOutputShaftLink);

	Vector3d rotarySpringOutputOffset = new Vector3d(0.0, 0.0, 1.5 * SEAParameters.MOTOR_BODY_RADIUS);
	actuatorOutputShaft = new PinJoint("actuatorOutputShaft" + suffix, rotarySpringOutputOffset, this, Joint.Z);

	gearboxOutputShaft.addJoint(actuatorOutputShaft);

	Link springOutputLink = getSpringOutputLink();
	actuatorOutputShaft.setLink(springOutputLink);

	instanceCounter++;
    }

    private Link getMotorBaseLink(double xOffset)
    {
	Link link = new Link("motorBase");

	LinkGraphics linkGraphics = new LinkGraphics();
	linkGraphics.translate(xOffset, 0.0, 0.0);

	link.setLinkGraphics(linkGraphics);

	link.setMass(1.0);
	link.setComOffset(0.0, 0.0, 0.0);
	link.setMomentOfInertia(1.0 * 1e-2 * 1e-2, 1.0 * 1e-2 * 1e-2, 1.0 * 1e-2 * 1e-2);

	if (SHOW_COORDINATE_SYSTEM)
	    linkGraphics.addCoordinateSystem(2.0 * SEAParameters.MOTOR_BODY_RADIUS);

	linkGraphics.addCylinder(SEAParameters.MOTOR_BODY_RADIUS, SEAParameters.MOTOR_BODY_RADIUS, YoAppearance.DarkRed());

	return link;
    }
    
    public void updateEncoderValues()
    {	
	//****gearbox***
	gearboxOutputEncoder.setActualPosition(gearboxOutputShaft.getQ().val);
	gearboxOutputPositionFromEncoder.val = gearboxOutputEncoder.getPositionFromEncoder();
	
	//set the ticks
	gearboxOutputEncoderTicks.set(gearboxOutputEncoder.getEncoderTicks());
	gearboxOutputEncoderStateMachineVelocityEstimator.update();
	
	//get the velocity
	double gearboxProcessedRate = gearboxOutputEncoderStateMachineVelocityEstimator.getProcessedRate().val;
	gearboxOutputVelocityFromEncoder.val = gearboxOutputEncoder.converTicksToDistance(gearboxProcessedRate);
	
	
	//***actuator***
	double actuatorOutputPositionRelativeToGround = gearboxOutputShaft.getQ().val + actuatorOutputShaft.getQ().val;
	actuatorOutputEncoder.setActualPosition(actuatorOutputPositionRelativeToGround);
	actuatorOutputPositionFromEncoder.val = actuatorOutputEncoder.getPositionFromEncoder();
	
	//Set the ticks
	actuatorOutputRealtiveToGroundEncoderTicks.set(actuatorOutputEncoder.getEncoderTicks());
	actuatorOutputEncoderStateMachineVelocityEstimator.update();
	

	//actuator velocity
	double actuatorProcessedRate = actuatorOutputEncoderStateMachineVelocityEstimator.getProcessedRate().val;
	actuatorOutputVelocityFromEncoder.val = actuatorOutputEncoder.converTicksToDistance(actuatorProcessedRate);
    }

    public double getMotorShaftPosition()
    {
	return motorShaftJoint.getQ().val;
    }

    public double getMotorShaftVelocity()
    {
	return motorShaftJoint.getQD().val;
    }

    public double getGearboxOutputShaftPosition()
    {
	return gearboxOutputShaft.getQ().val;
    }

    public double getGearboxInputShaftPosition()
    {
	return getGearboxOutputShaftPosition() * SEAParameters.GEAR_RATIO;
    }

    public double getGearboxOutputShaftVelocity()
    {
	return gearboxOutputShaft.getQD().val;
    }

    public double getGearboxInputShaftVelocity()
    {
	return getGearboxOutputShaftVelocity() * SEAParameters.GEAR_RATIO;
    }

    public double getSpringDeflection()
    {
	return actuatorOutputShaft.getQ().val;
    }

    public double getSpringDeflectionRate()
    {
	return actuatorOutputShaft.getQD().val;
    }

    public double getActuatorOutputPosition()
    {
	return (gearboxOutputShaft.getQ().val + actuatorOutputShaft.getQ().val);
    }

    public double getActuatorOutputVelocity()
    {
	return (gearboxOutputShaft.getQD().val + actuatorOutputShaft.getQD().val);
    }

    public double getTorqueOnMotorMass()
    {
	return motorShaftJoint.getTau().val;
    }
    
    //***Encoder gets
    
    
    //gearbox
    public double getGearboxOutputShaftPositionFromEncoder()
    {
	return gearboxOutputPositionFromEncoder.val;
    }
    
    public double getGearboxOutputShaftVelocityFromEncoder()
    {
	return gearboxOutputVelocityFromEncoder.val;
    }
    
    //actuator
    public double getActuatorOutputPositionFromEncoder()
    {
	return actuatorOutputPositionFromEncoder.val;
    }
    
    public double getActuatorOutputVelocityFromEncoder()
    {
	return actuatorOutputVelocityFromEncoder.val;
    }
    
    //Spring 
    public double getSpringDeflectionFromEncoder()
    {
	return (getActuatorOutputPositionFromEncoder() - getGearboxOutputShaftPositionFromEncoder());
    }
    
    public double getSpringDeflectionRateFromEncoder()
    {
	return (getActuatorOutputVelocityFromEncoder() - getGearboxOutputShaftVelocityFromEncoder());
    }
    //**End encoder section
    

    public void setTorqueOnMotorMass(double torque)
    {
	motorShaftJoint.setTau(torque);
    }

    public double getTorqueOnGearboxMass()
    {
	return gearboxOutputShaft.getTau().val;
    }

    public void setTorqueOnGearboxMass(double torque)
    {
	gearboxOutputShaft.setTau(torque);
    }

    public double getTorqueOnLoadMass()
    {
	return actuatorOutputShaft.getTau().val;
    }

    public void setTorqueOnLoadMass(double torque)
    {
	actuatorOutputShaft.setTau(torque);
    }

    private Link getMotorShaftLink()
    {
	Link link = new Link("motorShaft");

	LinkGraphics linkGraphics = new LinkGraphics();

	link.setLinkGraphics(linkGraphics);

	link.setMass(SEAParameters.MOTOR_ROTOR_MASS);
	link.setComOffset(0.0, 0.0, 0.0);

	// the only number really matters is z, but include x and y anyway
	link.setMomentOfInertia(SEAParameters.MOTOR_ROTOR_I, SEAParameters.MOTOR_ROTOR_I, SEAParameters.MOTOR_ROTOR_I);

	if (SHOW_COORDINATE_SYSTEM)
	    linkGraphics.addCoordinateSystem(2.0 * SEAParameters.MOTOR_BODY_RADIUS);

	linkGraphics.addCylinder(SEAParameters.MOTOR_BODY_RADIUS, SEAParameters.MOTOR_SHAFT_RADIUS, YoAppearance.Yellow());

	return link;
    }

    private Link getGearboxShaftLink()
    {
	Link link = new Link("gearboxdInputShaft");

	LinkGraphics linkGraphics = new LinkGraphics();

	link.setLinkGraphics(linkGraphics);

	link.setMass(SEAParameters.GEARBOX_OUTPUT_MASS);
	link.setComOffset(0.0, 0.0, 0.0);
	link.setMomentOfInertia(SEAParameters.GEARBOX_OUTPUT_I, SEAParameters.GEARBOX_OUTPUT_I, SEAParameters.GEARBOX_OUTPUT_I);

	if (SHOW_COORDINATE_SYSTEM)
	    linkGraphics.addCoordinateSystem(2.0 * SEAParameters.MOTOR_BODY_RADIUS);

	linkGraphics.addCylinder(SEAParameters.MOTOR_BODY_RADIUS, SEAParameters.MOTOR_SHAFT_RADIUS, YoAppearance.Fuchsia());

	return link;
    }

    private Link getSpringOutputLink()
    {
	Link link = new Link("SpringOutputLink");

	LinkGraphics linkGraphics = new LinkGraphics();

	link.setLinkGraphics(linkGraphics);

	link.setMass(SEAParameters.LOAD_MASS);
	link.setComOffset(0.0, 0.0, 0.0);

	double totalRotationalInertia = SEAParameters.LOAD_INERTIA;
	link.setMomentOfInertia(totalRotationalInertia, totalRotationalInertia, totalRotationalInertia);

	if (SHOW_COORDINATE_SYSTEM)
	    linkGraphics.addCoordinateSystem(2.0 * SEAParameters.MOTOR_BODY_RADIUS);

	linkGraphics.addCylinder(SEAParameters.MOTOR_BODY_RADIUS, SEAParameters.MOTOR_SHAFT_RADIUS, YoAppearance.Aqua());

	return link;
    }

    public ArrayList<DynamicGraphicObjectsList> getDynamicGraphicObjectsList()
    {
	return dynamicGraphicObjectsList;
    }
}
