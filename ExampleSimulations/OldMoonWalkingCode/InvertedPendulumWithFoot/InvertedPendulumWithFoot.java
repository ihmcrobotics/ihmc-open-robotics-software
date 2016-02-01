package us.ihmc.moonwalking.models.InvertedPendulumWithFoot;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
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
public class InvertedPendulumWithFoot extends Robot
{
   private final boolean SHOW_COORDINATE_SYSTEM = true;
   public static final boolean SHOW_MASS_BASED_GRAPHICS = true;


   private final double pendulumMass;
   private final double footMass;
   private final double gravity;

   private final double pendulumLength;

   private final double footWidth;
   private final double footLength;

   private ArrayList<DynamicGraphicObject> forces = new ArrayList();
   private ArrayList<DynamicGraphicObjectsList> dynamicGraphicObjectsList = new ArrayList<DynamicGraphicObjectsList>();

   private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<GroundContactPoint>();

   private final FloatingPlanarJoint bodyJoint;
   private final PinJoint ankleJoint;

   private YoVariable bodyPitch;
   private YoVariable bodyPitchDot;


   public InvertedPendulumWithFoot(double gravity, double pendulumLength, double pendulumMass)
   {
      super("IP_With_Foot");
      this.pendulumMass = pendulumMass;
      this.footMass = pendulumMass / 10.0;
      this.pendulumLength = pendulumLength;
      this.gravity = gravity;

      this.footWidth = pendulumLength / 4.0;
      this.footLength = pendulumLength / 2.0;

      this.setGravity(0.0, 0.0, -gravity);

      bodyJoint = new FloatingPlanarJoint("body", this);

      Link bodyLink = body();
      bodyJoint.setLink(bodyLink);


      bodyJoint.setDynamic(true);
      this.addRootJoint(bodyJoint);


      ankleJoint = new PinJoint("ankle", new Vector3d(0.0, 0.0, -pendulumLength), this, Axis.Y);
      Link footLink = getFootLink();

      bodyJoint.addJoint(ankleJoint);

      ankleJoint.setLink(footLink);


      addFootGroundContactPoints(ankleJoint);
      addBodyGroundContactPoints(bodyJoint);

      DynamicGraphicObjectsList forceList = new DynamicGraphicObjectsList("forces", forces);
      dynamicGraphicObjectsList.add(forceList);

      setInitialAngle(0.0);

      bodyPitch = this.getVariable("q_pitch");
      bodyPitchDot = this.getVariable("qd_pitch");
   }

   public double getPendulumLength()
   {
      return pendulumLength;
   }

   public double getPendulumMass()
   {
      return pendulumMass;
   }

   public double getFootLength()
   {
      return footLength;
   }

   public double getGravity()
   {
      return gravity;
   }

   public double getTime()
   {
      return t.getDoubleValue();
   }

   public void setTime(double time)
   {
      t.setValueFromDouble(time);
   }

   public double getAngularPosition()
   {
      return ankleJoint.getQ().getDoubleValue();
   }

   public double getAngularVelocity()
   {
      return ankleJoint.getQD().getDoubleValue();
   }

   public void setTorque(double torque)
   {
      ankleJoint.setTau(torque);
   }

   public void setPitchAndPitchDot(double pitch, double pitchDot)
   {
      bodyJoint.setRotation(pitch, pitchDot);
   }


   public ArrayList<DynamicGraphicObjectsList> getDynamicGraphicObjectsList()
   {
      return dynamicGraphicObjectsList;
   }

   public void setInitialAngle(double angle)
   {
      setInitialAngleAndVelocity(angle, 0.0);
   }

   public double getBodyPitch()
   {
      return bodyPitch.getValueAsDouble();
   }

   public double getBodyPitchDot()
   {
      return bodyPitchDot.getValueAsDouble();
   }



   public void setInitialAngleAndVelocity(double angle, double angularVelocity)
   {
//    ankleJoint.getQ().setValueAsDouble(angle);
//    ankleJoint.getQD().setValueAsDouble(angularVelocity);

      ankleJoint.setInitialState(-angle, -angularVelocity);


      bodyJoint.setRotation(angle, angularVelocity);


      double x = 0.0;    // pendulumLength * Math.sin(angle);
      double z = pendulumLength * (Math.cos(angle));

      double xdot = pendulumLength * angularVelocity * Math.cos(angle);
      double zdot = -pendulumLength * Math.abs(angularVelocity) * Math.sin(Math.abs(angle));


      bodyJoint.setCartesianPosition(x, z, xdot, zdot);



      resetGroundContacts();
   }

   private void resetGroundContacts()
   {
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         groundContactPoint.setForce(0.0, 0.0, 0.0);
      }
   }


   private Link body()
   {
      Link link = new Link("body");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      link.setLinkGraphics(linkGraphics);

      link.setMass(pendulumMass);
      link.setComOffset(0.0, 0.0, 0.0);
      link.setMomentOfInertia(pendulumLength * 1e-2 * 1e-2, pendulumLength * 1e-2 * 1e-2, pendulumLength * 1e-2 * 1e-2);

      if (SHOW_COORDINATE_SYSTEM)
         linkGraphics.addCoordinateSystem(pendulumLength / 2.0);

      linkGraphics.addSphere(pendulumLength / 5.0, YoAppearance.Yellow());
      linkGraphics.addCylinder(-pendulumLength, pendulumLength / 20.0, YoAppearance.DarkRed());

      return link;
   }

   private Link getFootLink()
   {
      Link link = new Link("foot");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      link.setLinkGraphics(linkGraphics);

      link.setMass(footMass);
      link.setComOffset(0.0, 0.0, 0.0);
      link.setMomentOfInertia(footMass * 0.1 * 0.1, footMass * 0.1 * 0.1, footMass * 0.1 * 0.1);
      if (SHOW_COORDINATE_SYSTEM)
         linkGraphics.addCoordinateSystem(pendulumLength / 2.0);

      linkGraphics.addCube(footLength, footWidth, pendulumLength / 30.0, YoAppearance.Olive());

      return link;
   }

   private void addFootGroundContactPoints(Joint parentJoint)
   {
      addGroundContactPoint("FL", new Vector3d(footLength / 2.0, footWidth, 0.0), parentJoint);
      addGroundContactPoint("FR", new Vector3d(footLength / 2.0, -footWidth, 0.0), parentJoint);
      addGroundContactPoint("HR", new Vector3d(-footLength / 2.0, -footWidth, 0.0), parentJoint);
      addGroundContactPoint("HL", new Vector3d(-footLength / 2.0, footWidth, 0.0), parentJoint);
   }

   private void addBodyGroundContactPoints(Joint parentJoint)
   {
      addGroundContactPoint("bodySideRight", new Vector3d(pendulumLength / 5.0, 0.0, 0.0), parentJoint);
      addGroundContactPoint("bodySideLeft", new Vector3d(-pendulumLength / 5.0, 0.0, 0.0), parentJoint);
   }

   private void addGroundContactPoint(String pointName, Vector3d offset, Joint parentJoint)
   {
      GroundContactPoint contactPoint = new GroundContactPoint(pointName, offset, this);
      parentJoint.addGroundContactPoint(contactPoint);

      groundContactPoints.add(contactPoint);


      DynamicGraphicVector force = new DynamicGraphicVector("forceVector" + pointName, contactPoint, 0.001);
      forces.add(force);
   }


}
