package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import com.vividsolutions.jts.math.Vector3D;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters2;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters2.LinkNames;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class FourBarLinkageRobot extends Robot

{
			

	// Initialize variables

	public FourBarLinkageRobot(String name, FourBarLinkageParameters fourBarLinkageParameters, Vector3d offsetWorld) {
		super("fourBarLinkageRobot" + name);
		
		Vector3d offsetJoint12 = new Vector3d(Math.cos(fourBarLinkageParameters.angle_1) * fourBarLinkageParameters.linkageLength_1, 0.0, Math.sin(fourBarLinkageParameters.angle_1) * fourBarLinkageParameters.linkageLength_1);
		Vector3d offsetJoint23 = new Vector3d(-Math.sin(fourBarLinkageParameters.angle_1 + fourBarLinkageParameters.angle_2 ) * fourBarLinkageParameters.linkageLength_2, 0.0, -Math.cos(fourBarLinkageParameters.angle_1 + fourBarLinkageParameters.angle_2) * fourBarLinkageParameters.linkageLength_2);
		Vector3d offsetJoint34 = new Vector3d(-Math.sin(fourBarLinkageParameters.angle_1 + fourBarLinkageParameters.angle_2 + fourBarLinkageParameters.angle_3) * fourBarLinkageParameters.linkageLength_3, 0.0, -Math.cos(fourBarLinkageParameters.angle_1 + fourBarLinkageParameters.angle_2 + fourBarLinkageParameters.angle_3) * fourBarLinkageParameters.linkageLength_3);
		
		PinJoint rootPinJoint = new PinJoint("rootPinJoint", offsetWorld, this, Axis.Y);
		Link fourBarLink1 = fourBarLink("fourBarLink1" + name, fourBarLinkageParameters.linkageLength_1, fourBarLinkageParameters.radius_1, fourBarLinkageParameters.mass_1);
		Link fourBarLink1A = fourBarLink("fourBarLink1A" + name, fourBarLinkageParameters.linkageLength_4, fourBarLinkageParameters.radius_4, fourBarLinkageParameters.mass_4);
		rootPinJoint.setLink(fourBarLink1);
		rootPinJoint.setLink(fourBarLink1A);
		rootPinJoint.setDamping(fourBarLinkageParameters.damping_1);
		this.addRootJoint(rootPinJoint);
//		constraint1A = new ExternalForcePoint("constraint1A", new Vector3d(0.0, 0.0, -L3), this);
//		rootPinJoint.physics.addExternalForcePoint(constraint1A);
		
		PinJoint pinJoint2 = new PinJoint("pinJoint2", offsetJoint12 , this, Axis.Y);
		Link fourBarLink2 = fourBarLink("fourBarLink2" + name, fourBarLinkageParameters.linkageLength_2, fourBarLinkageParameters.radius_2, fourBarLinkageParameters.mass_2);
		pinJoint2.setLink(fourBarLink2);
		pinJoint2.setDamping(fourBarLinkageParameters.damping_2);
		rootPinJoint.addJoint(pinJoint2);
		
		PinJoint pinJoint3 = new PinJoint("pinJoint3", offsetJoint23 , this, Axis.Y);
		Link fourBarLink3 = fourBarLink("fourBarLink3" + name, fourBarLinkageParameters.linkageLength_3, fourBarLinkageParameters.radius_2, fourBarLinkageParameters.mass_2);
		pinJoint3.setLink(fourBarLink3);
		pinJoint3.setDamping(fourBarLinkageParameters.damping_3);
		pinJoint2.addJoint(pinJoint3);
		
		
	}

	public Link fourBarLink(String linkName, double length, double radius, double mass) {

		Link link = new Link(linkName);
		Matrix3d inertiaCylinder = createInertiaMatrixCylinder(linkName, length, radius, mass);
		link.setMomentOfInertia(inertiaCylinder);
		link.setMass(mass);
		link.setComOffset(new Vector3d(0.0, 0.0, length / 2.0));

		Graphics3DObject linkGraphics = new Graphics3DObject();
		linkGraphics.addCylinder(length, radius, YoAppearance.HotPink());
		link.setLinkGraphics(linkGraphics);
		link.addCoordinateSystemToCOM(0.15);
		return link;
	}

	private Matrix3d createInertiaMatrixCylinder(String linkName, double length, double radius, double mass) {
		Matrix3d inertiaCylinder = new Matrix3d();
		inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);
		return inertiaCylinder;
	}
}
