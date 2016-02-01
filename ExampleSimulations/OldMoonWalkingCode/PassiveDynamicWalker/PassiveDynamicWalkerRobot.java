package us.ihmc.moonwalking.models.PassiveDynamicWalker;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

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
public class PassiveDynamicWalkerRobot extends Robot
{
  private static final double GRAVITY = 9.81;

  private static final double THIGH_MASS = 2.4443;
  private static final double SHIN_MASS = 1.015;

  private static final double SHIN_COM = 0.1725;
  private static final double SHIN_LENGTH = 0.44;
  private static final double SHIN_OFFSET = 0.004;
  private static final double SHIN_RADIUS_GYRATION = 0.197;

  private static final double THIGH_COM = 0.091;
  private static final double THIGH_LENGTH = 0.35;
  private static final double THIGH_OFFSET = 0.0;
  private static final double THIGH_RADIUS_GYRATION = 0.0995;

  private static final double FOOT_RADIUS_CURVATURE = 0.2;
  private static final double FOOT_CENTER_X = 0.05;
  private static final double FOOT_CENTER_Z = -0.25;
  private static final double EPSILON_THIGH = 0.097;
  private static final double EPSILON_KNEE = 0.2;
  private static final double FOOT_ARC = 1.5;
  private static final double THIGH_MOI_Y = THIGH_MASS*THIGH_RADIUS_GYRATION*THIGH_RADIUS_GYRATION, SHIN_MOI_Y = SHIN_MASS*SHIN_RADIUS_GYRATION*SHIN_RADIUS_GYRATION;

  //only for graphics
  private static final double BODY_RADIUS = 0.1;
  private static final double SHIN_RADIUS = 0.03;
  private static final double THIGH_RADIUS = 0.045;

  private double initialStanceAngle = 0.2439;
  private double initialThighAngle = 0.2439;

  private final FloatingPlanarJoint plane;
  private final PinJoint hipLeft, hipRight, kneeLeft, kneeRight;

  public int groundContactpoints = 20;
  public ArrayList gcPoints = new ArrayList(2*groundContactpoints);

  private final double initialStanceAngularVelocity = 1.049;
  private final double initialThighAngularVelocity = -1.049;

  private final double hipOffsets = 0.05;

  public PassiveDynamicWalkerRobot()
  {
    super("PassiveDynamicWalker");
    this.setGravity(0.0, 0.0, -GRAVITY);

    plane = new FloatingPlanarJoint("plane", this, FloatingPlanarJoint.XZ);
    Link body = body();
    plane.setLink(body);
    this.addRootJoint(plane);

    double initialBodyXVelocity = 0.6;
    double initialBodyHeight = 0.8;
    plane.setCartesianPosition(0.0, initialBodyHeight, initialBodyXVelocity, 0.0);
    plane.setRotation(PassiveDynamicWalkerSimulation.GROUND_ANGLE_OF_INCLINATION, 0.0);

    hipLeft = new PinJoint("hipLeft", new Vector3d(0.0, hipOffsets, 0.0), this, Joint.Y);
    Link thigh = thigh();
    hipLeft.setLink(thigh);
    plane.addJoint(hipLeft);
    hipLeft.setInitialState(-(initialStanceAngle-EPSILON_THIGH), initialStanceAngularVelocity);

    hipRight = new PinJoint("hipRight", new Vector3d(0.0, -hipOffsets, 0.0), this, Joint.Y);
    hipRight.setLink(thigh);
    plane.addJoint(hipRight);
    hipRight.setInitialState(initialThighAngle+EPSILON_THIGH, initialThighAngularVelocity);

    kneeLeft = new PinJoint("kneeLeft", new Vector3d(0.0, 0.0, -THIGH_LENGTH), this, Joint.Y);
    Link shin = shin();
    kneeLeft.setLink(shin);
    hipLeft.addJoint(kneeLeft);
    kneeLeft.setLimitStops(0.0, 1.2, 1000.0, 10.0);

    kneeRight = new PinJoint("kneeRight", new Vector3d(0.0, 0.0, -THIGH_LENGTH), this, Joint.Y);
    kneeRight.setLink(shin);
    hipRight.addJoint(kneeRight);
    kneeRight.setLimitStops(0.0, 1.2, 1000.0, 10.0);

    for(int i=0;i<groundContactpoints;i++)
    {
      GroundContactPoint gc_heel_left = new GroundContactPoint("gc_heel_"+i+"_left", new Vector3d(FOOT_CENTER_X-FOOT_RADIUS_CURVATURE*Math.sin(0.25-i*(FOOT_ARC/groundContactpoints)), 0.0, FOOT_CENTER_Z-FOOT_RADIUS_CURVATURE*Math.cos(0.25-i*(FOOT_ARC/groundContactpoints))), this);
      GroundContactPoint gc_heel_right = new GroundContactPoint("gc_heel_"+i+"_right", new Vector3d(FOOT_CENTER_X-FOOT_RADIUS_CURVATURE*Math.sin(0.25-i*(FOOT_ARC/groundContactpoints)), 0.0, FOOT_CENTER_Z-FOOT_RADIUS_CURVATURE*Math.cos(0.25-i*(FOOT_ARC/groundContactpoints))), this);
      kneeLeft.addGroundContactPoint(gc_heel_left);
      kneeRight.addGroundContactPoint(gc_heel_right);
      gcPoints.add(gc_heel_left);
      gcPoints.add(gc_heel_right);
    }
  }

  private Link body()
  {
    Link ret = new Link("body");

    LinkGraphics linkGraphics = new LinkGraphics();
    ret.setLinkGraphics(linkGraphics);

    linkGraphics.addCoordinateSystem(0.15);
    ret.setMass(0.00005);
    ret.setComOffset(0.0, 0.0, 0.0);
    ret.setMomentOfInertia(0.0, 0.1, 0.0);
    linkGraphics.addSphere(BODY_RADIUS, YoAppearance.Red());
    return ret;
  }


  private Link thigh()
  {
    Link ret = new Link("thigh");

    LinkGraphics linkGraphics = new LinkGraphics();
    ret.setLinkGraphics(linkGraphics);
    ret.setMass(THIGH_MASS);
    ret.setComOffset(-THIGH_OFFSET, 0.0, -THIGH_COM);
    ret.setMomentOfInertia(0.0, THIGH_MOI_Y, 0.0);
    linkGraphics.addCylinder(-THIGH_LENGTH, THIGH_RADIUS);
    return ret;
  }

  private Link shin()
  {
    Link ret = new Link("shin");

    LinkGraphics linkGraphics = new LinkGraphics();
    ret.setLinkGraphics(linkGraphics);
    ret.setMass(SHIN_MASS);
    ret.setComOffset(SHIN_OFFSET, 0.0, -SHIN_COM);
    ret.setMomentOfInertia(0.0, SHIN_MOI_Y, 0.0);
    linkGraphics.addCylinder(-SHIN_LENGTH, SHIN_RADIUS, YoAppearance.Green());

    for(int i=0;i<groundContactpoints;i++)
    {
      linkGraphics.translate(FOOT_CENTER_X-FOOT_RADIUS_CURVATURE*Math.sin(0.25-i*(FOOT_ARC/groundContactpoints)), 0.0, FOOT_CENTER_Z-FOOT_RADIUS_CURVATURE*Math.cos(0.25-i*(FOOT_ARC/groundContactpoints)));
      linkGraphics.addSphere(0.01);
      linkGraphics.translate(-FOOT_CENTER_X+FOOT_RADIUS_CURVATURE*Math.sin(0.25-i*(FOOT_ARC/groundContactpoints)), 0.0, -FOOT_CENTER_Z+FOOT_RADIUS_CURVATURE*Math.cos(0.25-i*(FOOT_ARC/groundContactpoints)));
    }

    return ret;
  }

}
