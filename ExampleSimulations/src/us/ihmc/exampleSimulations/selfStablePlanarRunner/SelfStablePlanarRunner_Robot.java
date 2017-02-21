package us.ihmc.exampleSimulations.selfStablePlanarRunner;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class SelfStablePlanarRunner_Robot extends Robot implements RobotController {

    private static final long serialVersionUID = -7398771951537758129L;

    private final YoVariableRegistry registry = new YoVariableRegistry("RobotRegistry");
    
    private final YoVariableRegistry paramsReg  = new YoVariableRegistry("RobotParameters");
    private final DoubleYoVariable thighLength  = new DoubleYoVariable("thighLength" , paramsReg);
    private final DoubleYoVariable shinLength   = new DoubleYoVariable("shinLength"  , paramsReg);
    private final DoubleYoVariable bodyToWheel  = new DoubleYoVariable("bodyToWheel" , paramsReg);
    private final DoubleYoVariable wheelR       = new DoubleYoVariable("wheelR"      , paramsReg);
    private final DoubleYoVariable pendulumComZ = new DoubleYoVariable("pendulumComZ", paramsReg);
    private final DoubleYoVariable pendulumMass = new DoubleYoVariable("pendulumMass", paramsReg);
    private final DoubleYoVariable bodyMass     = new DoubleYoVariable("bodyMass"    , paramsReg);
    private final DoubleYoVariable wheelMass    = new DoubleYoVariable("wheelMass"   , paramsReg);
    private final DoubleYoVariable thighMass    = new DoubleYoVariable("thighMass"   , paramsReg);
    private final DoubleYoVariable shinMass     = new DoubleYoVariable("shinMass"    , paramsReg);
    
    private SliderJoint bodyJoint_T1, bodyJoint_T2;
    private PinJoint bodyJoint_R;
    private SliderJoint pendulumJoint;
    private PinJoint leftUpperHip, rightUpperHip;
    private SliderJoint leftMidHip, rightMidHip;
    public PinJoint drivingWheel;
    private SliderJoint leftKnee, rightKnee;
    
    private ExternalForcePoint wheelOnLLeg, lLegOnWheel, wheelOnRLeg, rLegOnWheel;

    private GroundContactPoint leftGC, rightGC;

    private DoubleYoVariable[][] footForces;
    private DoubleYoVariable[] bodyVelocityInWorld;
    
    public SelfStablePlanarRunner_Robot(String name, double pendulumComZ, double q_y0) {
	super(name);
	this.getYoVariableRegistry().addChild(paramsReg);
	
	this.pendulumComZ.set(pendulumComZ);
	thighLength .set(  1.5);
	shinLength  .set(  0.6);
	bodyToWheel .set(  0.8);
	wheelR      .set(  0.25);
	bodyMass    .set(  1.0);
	pendulumMass.set( 50.0);
	wheelMass   .set(  1.0);
	thighMass   .set(  1.0);
	shinMass    .set(  2.0);
	

	bodyJoint_T1 = new SliderJoint("x"    , new Vector3D(0.0, q_y0, 0.0), this, Axis.X);
	bodyJoint_T2 = new SliderJoint("z"    , new Vector3D(), this, Axis.Z);
	bodyJoint_R  = new PinJoint   ("pitch", new Vector3D(), this, Axis.Y);

	pendulumJoint = new SliderJoint("pendulum", new Vector3D(), this, Axis.Z);
	
	leftUpperHip  = new PinJoint("l_uhip", new Vector3D(0, 0.15, 0), this, Axis.Y);
	rightUpperHip = new PinJoint("r_uhip", new Vector3D(0,-0.15, 0), this, Axis.Y);

	leftMidHip  = new SliderJoint("l_mhip", new Vector3D(), this, Axis.Z);
	rightMidHip = new SliderJoint("r_mhip", new Vector3D(), this, Axis.Z);

	wheelOnLLeg = new ExternalForcePoint("efp_wheelOnLLeg", new Vector3D(0, 0, -bodyToWheel.getDoubleValue()), this);
	wheelOnRLeg = new ExternalForcePoint("efp_wheelOnRLeg", new Vector3D(0, 0, -bodyToWheel.getDoubleValue()), this);

	drivingWheel = new PinJoint("driving_wheel", new Vector3D(0, 0, -bodyToWheel.getDoubleValue()), this, Axis.Y);

	lLegOnWheel = new ExternalForcePoint("efp_lLegOnWheel", new Vector3D(0, 0, -wheelR.getDoubleValue()), this);
	rLegOnWheel = new ExternalForcePoint("efp_rLegOnWheel", new Vector3D(0, 0,  wheelR.getDoubleValue()), this);

	leftKnee  = new SliderJoint("l_knee", new Vector3D(0, 0, -thighLength.getDoubleValue()), this, Axis.Z);
	rightKnee = new SliderJoint("r_knee", new Vector3D(0, 0, -thighLength.getDoubleValue()), this, Axis.Z);

	leftGC  = new GroundContactPoint( "leftGC", new Vector3D(0, 0, -shinLength.getDoubleValue()), this);
	rightGC = new GroundContactPoint("rightGC", new Vector3D(0, 0, -shinLength.getDoubleValue()), this);
	
	bodyJoint_T1.setQd(3.);
	bodyJoint_R .setQ(Math.toRadians(3));
	drivingWheel.setQd(6.0);
	initWihtFootHeight(Math.toRadians(90), 0.05, 0.5);
	pendulumJoint.setQ(this.pendulumComZ.getDoubleValue());
	
	
	leftKnee .setLimitStops(0.0, shinLength.getDoubleValue(), 1000, 5000);
	rightKnee.setLimitStops(0.0, shinLength.getDoubleValue(), 1000, 5000);

	bodyJoint_T1.setLink(nullLink());
	bodyJoint_T2.setLink(nullLink());
	bodyJoint_R .setLink(bodyLink());
	pendulumJoint.setLink(pendulumLink());
	drivingWheel.setLink(drivingWheelLink());
	leftUpperHip.setLink(nullLink());	rightUpperHip.setLink(nullLink());
	leftMidHip  .setLink(upperLeg());	rightMidHip  .setLink(upperLeg());
	leftKnee    .setLink(lowerLeg());	rightKnee    .setLink(lowerLeg());

	drivingWheel.addExternalForcePoint(lLegOnWheel);	drivingWheel.addExternalForcePoint(rLegOnWheel);
	leftMidHip  .addExternalForcePoint(wheelOnLLeg);	rightMidHip.addExternalForcePoint(wheelOnRLeg);

	this.addRootJoint(bodyJoint_T1);
	bodyJoint_T1.addJoint(bodyJoint_T2);
	bodyJoint_T2.addJoint(bodyJoint_R);
	bodyJoint_R .addJoint(pendulumJoint);
	bodyJoint_R .addJoint(drivingWheel);
	bodyJoint_R .addJoint(leftUpperHip);	bodyJoint_R  .addJoint(rightUpperHip);
	leftUpperHip.addJoint(leftMidHip  );	rightUpperHip.addJoint(rightMidHip  );
	leftMidHip  .addJoint(leftKnee    );	rightMidHip  .addJoint(rightKnee    );

	leftKnee.addGroundContactPoint(leftGC);		rightKnee.addGroundContactPoint(rightGC);
	
	footForces = new DoubleYoVariable[][]{{leftGC.getYoForce().getYoX(), leftGC.getYoForce().getYoY(), leftGC.getYoForce().getYoZ()}, 
	      {rightGC.getYoForce().getYoX(), rightGC.getYoForce().getYoY(), rightGC.getYoForce().getYoZ()}};
	bodyVelocityInWorld = new DoubleYoVariable[]{bodyJoint_T1.getQDYoVariable(), bodyJoint_R.getQDYoVariable(), bodyJoint_T2.getQDYoVariable()};
	
	this.setController(this);
	initControl();
    }

    private Link bodyLink() {
	Link ret = new Link("body");
	ret.setMass(bodyMass.getDoubleValue());
	ret.setMomentOfInertia(0.001, 0.001, 0.001);
	ret.setComOffset(0.0, 0.0, 0.0);

	Graphics3DObject g = new Graphics3DObject();
	g.translate(0, 0.25, 0);
	g.rotate(Math.PI/2, Axis.X);
	g.addCylinder(0.5, 0.025, YoAppearance.Red());
	g.rotate(-Math.PI/2, Axis.X);
	g.translate(0,-0.25, 0);
	g.translate(0, 0.099/2.0, 0);
	g.rotate(Math.PI/2, Axis.X);
	g.addCylinder(0.099, 0.1, YoAppearance.DarkGreen());
	g.rotate(-Math.PI/2, Axis.X);
	g.translate(0, -0.099/2.0, 0);
	g.addCube(0.2, 0.1, pendulumComZ.getDoubleValue(), YoAppearance.DarkGreen());

	ret.setLinkGraphics(g);

	return ret;
    }
    
    private Link pendulumLink() {
	Link ret = new Link("pendulum");
	ret.setMass(pendulumMass.getDoubleValue());
	ret.setMomentOfInertia(0.001, 0.001, 0.005);
	ret.setComOffset(0.0, 0.0, 0.0);

	Graphics3DObject g = new Graphics3DObject();
	g.translate(0, 0.055, 0);
	g.rotate(Math.PI/2, Axis.X);
	g.addCylinder(0.11, 0.3, YoAppearance.DarkKhaki());
	g.rotate(-Math.PI/2, Axis.X);

	ret.setLinkGraphics(g);

	return ret;
    }

    private Link drivingWheelLink() {
	Link ret = new Link("driving_wheel");
	ret.setMass(wheelMass.getDoubleValue());
	ret.setMomentOfInertia(0.05, 0.01, 0.05);
	ret.setComOffset(0.0, 0.0, 0.0);

	Graphics3DObject g = new Graphics3DObject();
	g.translate(0, 0.0505, 0);
	g.rotate(Math.PI/2, Axis.X);
	g.addArcTorus(0, 2.0*Math.PI, wheelR.getDoubleValue(), 0.025, YoAppearance.DarkCyan());
	g.rotate(-Math.PI/2, Axis.X);
	g.translate(0, 0, -wheelR.getDoubleValue());
	g.addSphere(0.04);
	g.translate(0, 0, 2.0*wheelR.getDoubleValue());
	g.addSphere(0.04);
	g.translate(-wheelR.getDoubleValue(), 0, -wheelR.getDoubleValue());
	g.addSphere(0.04);
	g.translate(2.0*wheelR.getDoubleValue(), 0, 0);
	g.addSphere(0.04);
	g.translate(-wheelR.getDoubleValue(), 0, 0);
	g.translate(0,-0.11, 0);
	g.rotate(Math.PI/2, Axis.X);
	g.addArcTorus(0, 2.0*Math.PI, wheelR.getDoubleValue(), 0.025, YoAppearance.DarkCyan());
	g.rotate(-Math.PI/2, Axis.X);
	g.translate(0, 0, -wheelR.getDoubleValue());
	g.addSphere(0.04);
	g.translate(0, 0, 2.0*wheelR.getDoubleValue());
	g.addSphere(0.04);
	g.translate(-wheelR.getDoubleValue(), 0, -wheelR.getDoubleValue());
	g.addSphere(0.04);
	g.translate(2.0*wheelR.getDoubleValue(), 0, 0);
	g.addSphere(0.04);
	g.translate(-wheelR.getDoubleValue(), 0, 0);
	
	ret.setLinkGraphics(g);

	return ret;
    }

    private Link upperLeg() {
	Link ret = new Link("upper_leg");
	ret.setMass(thighMass.getDoubleValue());
	ret.setMomentOfInertia(0.001, 0.001, 0.001);
	ret.setComOffset(0.0, 0.0, -thighLength.getDoubleValue()/2.0);

	Graphics3DObject g = new Graphics3DObject();
	g.addHemiEllipsoid(0.07, 0.07, 0.07);
	g.addCylinder(-thighLength.getDoubleValue(), 0.07, YoAppearance.DarkGray());

	ret.setLinkGraphics(g);

	return ret;
    }

    private Link lowerLeg() {
	Link ret = new Link("lower_leg");
	ret.setMass(shinMass.getDoubleValue());
	ret.setMomentOfInertia(0.001, 0.001, 0.001);
	ret.setComOffset(0.0, 0.0, -shinLength.getDoubleValue()/2.0);

	Graphics3DObject g = new Graphics3DObject();
	g.addCylinder(-shinLength.getDoubleValue()+0.04, 0.04, YoAppearance.DarkRed());
	g.translate(0, 0, -shinLength.getDoubleValue()+0.04);
	g.rotate(Math.PI,Axis.X);
	g.addHemiEllipsoid(0.04, 0.04, 0.04);

	ret.setLinkGraphics(g);

	return ret;
    }

    private Link nullLink() {
	Link ret = new Link("null");    // smallPiece();
	ret.setMass(0.0);
	ret.setMomentOfInertia(0.0, 0.0, 0.0);
	ret.setComOffset(0.0, 0.0, 0.0);

	return ret;
    }

    private void initControl() {
    }
    
    public void doControl() {
	pendulumJoint.setQ  (pendulumComZ.getDoubleValue());
	pendulumJoint.setQd (0);
	pendulumJoint.setQdd(0);
	
	double K = 20000.0, B = 700.0;

	Vector3D force = new Vector3D();
	
	force.setX(K * (lLegOnWheel.getX() - wheelOnLLeg.getX()) + B * (lLegOnWheel.getXVelocity() - wheelOnLLeg.getXVelocity()));
	force.setY(K * (lLegOnWheel.getY() - wheelOnLLeg.getY()) + B * (lLegOnWheel.getYVelocity() - wheelOnLLeg.getYVelocity()));
	force.setZ(K * (lLegOnWheel.getZ()- wheelOnLLeg.getZ()) + B * (lLegOnWheel.getZVelocity() - wheelOnLLeg.getZVelocity()));

	wheelOnLLeg.setForce(force);
	force.negate();
	lLegOnWheel.setForce(force);

	force.setX(K * (rLegOnWheel.getX() - wheelOnRLeg.getX()) + B * (rLegOnWheel.getXVelocity() - wheelOnRLeg.getXVelocity()));
	force.setY(K * (rLegOnWheel.getY() - wheelOnRLeg.getY()) + B * (rLegOnWheel.getYVelocity() - wheelOnRLeg.getYVelocity()));
	force.setZ(K * (rLegOnWheel.getZ() - wheelOnRLeg.getZ()) + B * (rLegOnWheel.getZVelocity() - wheelOnRLeg.getZVelocity()));
	wheelOnRLeg.setForce(force);
	
	force.negate();
	rLegOnWheel.setForce(force);
	
    }

    public double getFootForce (RobotSide rS, Axis axis) {
	return footForces[rS.ordinal()][axis.ordinal()].getDoubleValue();
    }
    public double getBodyVxWyVzInWorld (Axis axis) {
	return bodyVelocityInWorld[axis.ordinal()].getDoubleValue();
    }
    public double getBodyPitch () {
	return bodyJoint_R.getQYoVariable().getDoubleValue();
    }
    public double getBodyPitchVelocity () {
	return bodyJoint_R.getQDYoVariable().getDoubleValue();
    }
    public double getHipPitch (RobotSide rS) {
	if (rS == RobotSide.LEFT)
	    return leftUpperHip.getQYoVariable().getDoubleValue();
	else
	    return rightUpperHip.getQYoVariable().getDoubleValue();
    }
    public double getL0() {
	return thighLength.getDoubleValue() + shinLength.getDoubleValue();
    }
    
    public double compute_q_mhip(RobotSide rS, double drivingWheelAngle) {
	double ret = + bodyToWheel.getDoubleValue() - Math.sqrt(
		+ bodyToWheel.getDoubleValue() * bodyToWheel.getDoubleValue()
		+ wheelR.getDoubleValue()      * wheelR.getDoubleValue()
		- 2.0 * bodyToWheel.getDoubleValue() * rS.negateIfLeftSide(wheelR.getDoubleValue()) * Math.cos(drivingWheelAngle));
	
	return ret;
    }

    public double compute_q_uhip(RobotSide rS, double drivingWheelAngle) {
	double ret = rS.negateIfRightSide(Math.asin(
		+ Math.sin(drivingWheelAngle)
		* wheelR.getDoubleValue() / (bodyToWheel.getDoubleValue() - compute_q_mhip(rS, drivingWheelAngle))
		));
	
	return ret;
    }
    
    private void initWihtFootHeight (double drivingWheelAngle, double footHeight, double qd_z) {
	initWithDrivingWheelAngle(drivingWheelAngle);
	double body_z = footHeight
		+ Math.max(
		(thighLength.getDoubleValue() + shinLength.getDoubleValue() - compute_q_mhip(RobotSide.LEFT , drivingWheelAngle)) * Math.cos(compute_q_uhip(RobotSide.LEFT , drivingWheelAngle)),
		(thighLength.getDoubleValue() + shinLength.getDoubleValue() - compute_q_mhip(RobotSide.RIGHT, drivingWheelAngle)) * Math.cos(compute_q_uhip(RobotSide.RIGHT, drivingWheelAngle)));
	bodyJoint_T2.setQ(body_z);
	bodyJoint_T2.setQd(qd_z);
    }
    
    private void initWithDrivingWheelAngle (double drivingWheelAngle) {
	drivingWheel .setQ(drivingWheelAngle);
	leftUpperHip .setQ(compute_q_uhip(RobotSide.LEFT , drivingWheelAngle));
	rightUpperHip.setQ(compute_q_uhip(RobotSide.RIGHT, drivingWheelAngle));
	leftMidHip   .setQ(compute_q_mhip(RobotSide.LEFT , drivingWheelAngle));
	rightMidHip  .setQ(compute_q_mhip(RobotSide.RIGHT, drivingWheelAngle));
    }
    
    public void initialize() {
    }

    public YoVariableRegistry getYoVariableRegistry() {
	return registry;
    }

    public String getDescription() {
	return null;
    }

}
