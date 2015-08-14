package us.ihmc.moonwalking.models.OneLeggedHopper;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

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
public class OneLeggedHopperRobot extends Robot
{
  private final boolean SHOW_COORDINATE_SYSTEM = true;
  public static final boolean SHOW_MASS_BASED_GRAPHICS = true;

  public static final double hopperMass = 100.0;
  public static double legMass = hopperMass/100.0;
  public static final double nominalLegLength = 1.0;
  private final double hopperBodyRadius = nominalLegLength/2.0;

  private final double thighDiameter = hopperBodyRadius/10.0;

  private final double gravity;



  private ArrayList<DynamicGraphicObject> forces = new ArrayList();
  private ArrayList<DynamicGraphicObjectsList> dynamicGraphicObjectsList = new ArrayList<DynamicGraphicObjectsList> ();

  private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<GroundContactPoint> ();

  private final FloatingPlanarJoint bodyJoint;
  private final PinJoint hipJoint;
  private final SliderJoint legJoint;

  private YoVariable bodyPitch;
  private YoVariable bodyPitchDot;

  private GroundContactPoint footContactPoint;

  public OneLeggedHopperRobot(double gravity)
  {
    super("Hopper");
    this.gravity = gravity;

    this.setGravity(0.0, 0.0, -gravity);

    bodyJoint = new FloatingPlanarJoint("body", this);

    bodyJoint.setCartesianPosition(0.0, 1.2 * 1.5 * nominalLegLength);

    Link bodyLink = body();
    bodyJoint.setLink(bodyLink);

    bodyJoint.setDynamic(true);
    this.addRootJoint(bodyJoint);

    hipJoint = new PinJoint("hipJoint", new Vector3d(0.0, 0.0, 0.0), this, Joint.Y);

    bodyJoint.addJoint(hipJoint);

    Link thighLink = getThighLink();
    hipJoint.setLink(thighLink);

    Vector3d legSliderOffset = new Vector3d(0.0, 0.0, -nominalLegLength);
    legJoint = new SliderJoint("legJoint", legSliderOffset, this, Joint.Z);
    legJoint.setLimitStops(-nominalLegLength/4.0, nominalLegLength/4.0, 1000.0, 100.0);


    hipJoint.addJoint(legJoint);

    Link shankLink = getShankLink();
    legJoint.setLink(shankLink);

    addFootGroundContactPoint(legJoint);
    addBodyGroundContactPoints(bodyJoint);

    DynamicGraphicObjectsList forceList = new DynamicGraphicObjectsList("forces", forces);
    dynamicGraphicObjectsList.add(forceList);

//    setInitialAngle(0.0);

    bodyPitch = this.getVariable("q_pitch");
    bodyPitchDot = this.getVariable("qd_pitch");

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

  public double getHipAngle()
  {
    return hipJoint.getQ().val;
  }

  public double getHipAngleDot()
  {
    return hipJoint.getQD().val;
  }

  public double getLegLength()
  {
    return legJoint.getQ().val;
  }

  public double getLegLengthDot()
  {
    return legJoint.getQD().val;
  }


  public void setHipTorque(double torque)
  {
    hipJoint.setTau(torque);
  }

  public void setLegTorque(double torque)
  {
    legJoint.setTau(torque);
  }

  public boolean isFootOnGround()
  {
    return (footContactPoint.fs.val > 0.5);
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

    linkGraphics.addCube(hopperBodyRadius, hopperBodyRadius/4.0, hopperBodyRadius/4.0, YoAppearance.Yellow());
    return link;
  }

 private Link getThighLink()
 {
   Link link = new Link("thigh");

   LinkGraphics linkGraphics = new LinkGraphics();

   link.setLinkGraphics(linkGraphics);

   double thighMass = legMass/2.0;
   link.setMass(thighMass);
   link.setComOffset(0.0, 0.0, -nominalLegLength/2.0);
   link.setMomentOfInertia(0.5 * thighMass * (nominalLegLength/2.0) * (nominalLegLength/2.0),
                           0.5 * thighMass * (nominalLegLength/2.0) * (nominalLegLength/2.0),
                           thighMass * (nominalLegLength/10.0) * (nominalLegLength/10.0));

   double downTranslation = nominalLegLength;
   linkGraphics.translate(0.0, 0.0, -downTranslation);
   linkGraphics.addCylinder(nominalLegLength, thighDiameter, YoAppearance.Gray());
   linkGraphics.translate(0.0, 0.0, downTranslation);

   if (SHOW_COORDINATE_SYSTEM)
   {
     linkGraphics.translate(0.0, 0.0, -nominalLegLength/2.0);
     linkGraphics.addCoordinateSystem(nominalLegLength/4.0);
     linkGraphics.translate(0.0, 0.0, nominalLegLength/2.0);
   }

   return link;
 }

 private Link getShankLink()
 {
   Link link = new Link("shank");

   LinkGraphics linkGraphics = new LinkGraphics();

   link.setLinkGraphics(linkGraphics);

   double shankMass = legMass / 2.0;
   link.setMass(shankMass);
   link.setComOffset(0.0, 0.0, 0.0);
   link.setMomentOfInertia(0.5 * shankMass * (nominalLegLength / 2.0) * (nominalLegLength / 2.0),
                           0.5 * shankMass * (nominalLegLength / 2.0) * (nominalLegLength / 2.0),
                           shankMass * (nominalLegLength / 10.0) * (nominalLegLength / 10.0));

   linkGraphics.translate(0.0, 0.0, nominalLegLength/2.0);
   linkGraphics.addCylinder( -nominalLegLength, 0.7 * thighDiameter, YoAppearance.Fuchsia());

   if (SHOW_COORDINATE_SYSTEM)
   {
     linkGraphics.translate(0.0, 0.0, -nominalLegLength / 2.0);
     linkGraphics.addCoordinateSystem(nominalLegLength/4.0);
     linkGraphics.translate(0.0, 0.0, nominalLegLength / 2.0);
   }

   return link;
 }

 private void addFootGroundContactPoint(Joint parentJoint)
 {
   footContactPoint = new GroundContactPoint("Foot", new Vector3d(0.0, 0.0, -nominalLegLength/2.0), this);
   parentJoint.addGroundContactPoint(footContactPoint);

   groundContactPoints.add(footContactPoint);


   DynamicGraphicVector force = new DynamicGraphicVector(footContactPoint, 0.001);
   forces.add(force);
 }

 private void addBodyGroundContactPoints(Joint parentJoint)
 {
   addGroundContactPoint("bodySideRight",  new Vector3d(hopperBodyRadius, 0.0, 0.0), parentJoint);
   addGroundContactPoint("bodySideLeft",  new Vector3d(-hopperBodyRadius, 0.0, 0.0), parentJoint);
 }

 private void addGroundContactPoint(String pointName, Vector3d offset, Joint parentJoint)
 {
   GroundContactPoint contactPoint = new GroundContactPoint(pointName, offset, this);
   parentJoint.addGroundContactPoint(contactPoint);

   groundContactPoints.add(contactPoint);


   DynamicGraphicVector force = new DynamicGraphicVector(contactPoint, 0.001);
   forces.add(force);
 }

 public ArrayList<DynamicGraphicObjectsList> getDynamicGraphicObjectsList()
 {
   return dynamicGraphicObjectsList;
 }



}
