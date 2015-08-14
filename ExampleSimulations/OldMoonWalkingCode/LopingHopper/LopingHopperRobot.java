package us.ihmc.moonwalking.models.LopingHopper;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

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
public class LopingHopperRobot extends Robot
{
	private final boolean SHOW_COORDINATE_SYSTEM = true;
	public static final boolean SHOW_MASS_BASED_GRAPHICS = true;

	public static final double hopperMass = 100.0;
	public static double legMass = hopperMass / 100.0;
	public static final double nominalLegLength = 1.0;
	private final double hopperBodyRadius = nominalLegLength / 4.0;

	private final double thighDiameter = hopperBodyRadius / 10.0;

	private final double gravity;

	private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<GroundContactPoint>();

	private final FloatingPlanarJoint bodyJoint;
	private final PinJoint frontHipJoint;
	private final SliderJoint frontLegJoint;

	private final PinJoint backHipJoint;
	private final SliderJoint backLegJoint;

	private YoVariable bodyPitch;
	private YoVariable bodyPitchDot;

	private YoVariable q_frontHipJoint;
	private YoVariable q_backHipJoint;

	private GroundContactPoint frontFootContactPoint;
	private GroundContactPoint backFootContactPoint;

	public LopingHopperRobot(double gravity)
	{
		super("Hopper");
		this.gravity = gravity;

		this.setGravity(0.0, 0.0, -gravity);

		bodyJoint = new FloatingPlanarJoint("body", this);

		bodyJoint.setCartesianPosition(0.0, 1.5 * nominalLegLength);

		Link bodyLink = body();
		bodyJoint.setLink(bodyLink);

		bodyJoint.setDynamic(true);
		this.addRootJoint(bodyJoint);

		frontHipJoint = new PinJoint("frontHipJoint", new Vector3d(0.0, 0.0, 0.0), this, Joint.Y);

		bodyJoint.addJoint(frontHipJoint);

		Link frontThighLink = getThighLink("frontThigh");
		frontHipJoint.setLink(frontThighLink);

		Vector3d frontLegSliderOffset = new Vector3d(0.0, 0.0, -nominalLegLength);
		frontLegJoint = new SliderJoint("frontLegJoint", frontLegSliderOffset, this, Joint.Z);
		frontLegJoint.setLimitStops(-nominalLegLength / 4.0, nominalLegLength / 4.0, 1000.0, 100.0);

		frontHipJoint.addJoint(frontLegJoint);

		Link frontShankLink = getShankLink("frontShank");
		frontLegJoint.setLink(frontShankLink);

		backHipJoint = new PinJoint("backHipJoint", new Vector3d(0.0, 0.0, 0.0), this, Joint.Y);

		bodyJoint.addJoint(backHipJoint);

		Link backThighLink = getThighLink("backThigh");
		backHipJoint.setLink(backThighLink);

		Vector3d backLegSliderOffset = new Vector3d(0.0, 0.0, -nominalLegLength);
		backLegJoint = new SliderJoint("backLegJoint", backLegSliderOffset, this, Joint.Z);
		backLegJoint.setLimitStops(-nominalLegLength / 4.0, nominalLegLength / 4.0, 1000.0, 100.0);

		backHipJoint.addJoint(backLegJoint);

		Link backShankLink = getShankLink("backShank");
		backLegJoint.setLink(backShankLink);

		addFrontFootGroundContactPoint(frontLegJoint);
		addBackFootGroundContactPoint(backLegJoint);
		addBodyGroundContactPoints(bodyJoint);

		// setInitialAngle(0.0);

		bodyPitch = this.getVariable("q_pitch");
		bodyPitchDot = this.getVariable("qd_pitch");
		q_frontHipJoint = this.getVariable("q_frontHipJoint");
		q_backHipJoint = this.getVariable("q_backHipJoint");

		q_frontHipJoint.val = -0.3;
		q_backHipJoint.val = 0.3;


	}

	public double getGravity()
	{
		return gravity;
	}

	public double getBodyPitch()
	{
		return bodyPitch.val;
	}

	public double getBodyPitchDot()
	{
		return bodyPitchDot.val;
	}

	public YoVariable getTimeVariable()
	{
		return this.t;
	}

	public double getFrontHipAngle()
	{
		return frontHipJoint.getQ().val;
	}

	public double getFrontHipAngleDot()
	{
		return frontHipJoint.getQD().val;
	}

	public double getFrontLegLength()
	{
		return frontLegJoint.getQ().val;
	}

	public double getFrontLegLengthDot()
	{
		return frontLegJoint.getQD().val;
	}

	public void setFrontHipTorque(double torque)
	{
		frontHipJoint.setTau(torque);
	}

	public void setFrontLegTorque(double torque)
	{
		frontLegJoint.setTau(torque);
	}

	public boolean isFrontFootOnGround()
	{
		return (frontFootContactPoint.fs.val > 0.5);
	}

	public double getBackHipAngle()
	{
		return backHipJoint.getQ().val;
	}

	public double getBackHipAngleDot()
	{
		return backHipJoint.getQD().val;
	}

	public double getBackLegLength()
	{
		return backLegJoint.getQ().val;
	}

	public double getBackLegLengthDot()
	{
		return backLegJoint.getQD().val;
	}

	public void setBackHipTorque(double torque)
	{
		backHipJoint.setTau(torque);
	}

	public void setBackLegTorque(double torque)
	{
		backLegJoint.setTau(torque);
	}

	public boolean isBackFootOnGround()
	{
		return (backFootContactPoint.fs.val > 0.5);
	}

	private Link body()
	{
		Link link = new Link("body");

		LinkGraphics linkGraphics = new LinkGraphics();

		link.setLinkGraphics(linkGraphics);

		link.setMass(hopperMass);
		link.setComOffset(0.0, 0.0, 0.0);
		link.setMomentOfInertia(hopperMass * hopperBodyRadius * hopperBodyRadius, hopperMass * hopperBodyRadius * hopperBodyRadius, hopperMass * hopperBodyRadius * hopperBodyRadius);

		if (SHOW_COORDINATE_SYSTEM)
			linkGraphics.addCoordinateSystem(hopperBodyRadius * 1.2);

		linkGraphics.addCube(hopperBodyRadius, hopperBodyRadius*1.2, hopperBodyRadius);
		linkGraphics.addSphere(hopperBodyRadius);
		return link;
	}

	private Link getThighLink(String name)
	{
		Link link = new Link(name);

		LinkGraphics linkGraphics = new LinkGraphics();

		link.setLinkGraphics(linkGraphics);

		double thighMass = legMass / 2.0;
		link.setMass(thighMass);
		link.setComOffset(0.0, 0.0, -nominalLegLength / 2.0);
		link.setMomentOfInertia(0.5 * thighMass * (nominalLegLength / 2.0) * (nominalLegLength / 2.0), 0.5 * thighMass * (nominalLegLength / 2.0) * (nominalLegLength / 2.0), thighMass * (nominalLegLength / 10.0) * (nominalLegLength / 10.0));

		double downTranslation = nominalLegLength;
		linkGraphics.translate(0.0, 0.0, -downTranslation);
		linkGraphics.addCylinder(nominalLegLength, thighDiameter);
		linkGraphics.translate(0.0, 0.0, downTranslation);

		if (SHOW_COORDINATE_SYSTEM)
		{
			linkGraphics.translate(0.0, 0.0, -nominalLegLength / 2.0);
			linkGraphics.addCoordinateSystem(nominalLegLength / 4.0);
			linkGraphics.translate(0.0, 0.0, nominalLegLength / 2.0);
		}

		return link;
	}

	private Link getShankLink(String name)
	{
		Link link = new Link(name);

		LinkGraphics linkGraphics = new LinkGraphics();

		link.setLinkGraphics(linkGraphics);

		double shankMass = legMass / 2.0;
		link.setMass(shankMass);
		link.setComOffset(0.0, 0.0, 0.0);
		link.setMomentOfInertia(0.5 * shankMass * (nominalLegLength / 2.0) * (nominalLegLength / 2.0), 0.5 * shankMass * (nominalLegLength / 2.0) * (nominalLegLength / 2.0), shankMass * (nominalLegLength / 10.0) * (nominalLegLength / 10.0));

		linkGraphics.translate(0.0, 0.0, -nominalLegLength / 2.0);
		linkGraphics.addCylinder(nominalLegLength, 0.7 * thighDiameter);

		if (SHOW_COORDINATE_SYSTEM)
		{
			// linkGraphics.translate(0.0, 0.0, -nominalLegLength / 2.0);
			linkGraphics.addCoordinateSystem(nominalLegLength / 4.0);
			linkGraphics.translate(0.0, 0.0, nominalLegLength / 2.0);
		}

		return link;
	}

	private void addFrontFootGroundContactPoint(Joint parentJoint)
	{
		frontFootContactPoint = new GroundContactPoint("frontFoot", new Vector3d(0.0, 0.0, -nominalLegLength / 2.0), this);
		parentJoint.addGroundContactPoint(frontFootContactPoint);

		groundContactPoints.add(frontFootContactPoint);
	}

	private void addBackFootGroundContactPoint(Joint parentJoint)
	{
		backFootContactPoint = new GroundContactPoint("backFoot", new Vector3d(0.0, 0.0, -nominalLegLength / 2.0), this);
		parentJoint.addGroundContactPoint(backFootContactPoint);

		groundContactPoints.add(backFootContactPoint);

	}

	private void addBodyGroundContactPoints(Joint parentJoint)
	{
		addGroundContactPoint("bodySideRight", new Vector3d(hopperBodyRadius, 0.0, 0.0), parentJoint);
		addGroundContactPoint("bodySideLeft", new Vector3d(-hopperBodyRadius, 0.0, 0.0), parentJoint);
	}

	private void addGroundContactPoint(String pointName, Vector3d offset, Joint parentJoint)
	{
		GroundContactPoint contactPoint = new GroundContactPoint(pointName, offset, this);
		parentJoint.addGroundContactPoint(contactPoint);

		groundContactPoints.add(contactPoint);
	}

}
