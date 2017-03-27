package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class PointMassRobot extends Robot
{
   private static final double DEFAULT_MASS = 10.0;
   private static final double DEFAULT_RADIUS = 0.02;
   private static final double DEFAULT_FORCE_VECTOR_SCALE = 1.0 / 50.0;

   private final SliderJoint xJoint, yJoint, zJoint;
   private final ExternalForcePoint externalForcePoint;

   private final Link zLink;

   public PointMassRobot()
   {
      this("PointMassRobot");
   }

   public PointMassRobot(String name)
   {
      this(name, DEFAULT_MASS, DEFAULT_RADIUS, DEFAULT_FORCE_VECTOR_SCALE);
   }

   public PointMassRobot(String name, double mass, double radius, double forceVectorScale)
   {
      super(name);

      this.setGravity(0.0);

      xJoint = new SliderJoint("pointMassX", new Vector3D(), this, Axis.X);
      Link xLink = new Link("xLink");
      xJoint.setLink(xLink);

      yJoint = new SliderJoint("pointMassY", new Vector3D(), this, Axis.Y);
      Link yLink = new Link("yLink");
      yJoint.setLink(yLink);

      zJoint = new SliderJoint("pointMassZ", new Vector3D(), this, Axis.Z);
      zLink = new Link("zLink");
      zLink.setMass(mass);
      zLink.setMomentOfInertia(0.0, 0.0, 0.0);
      Graphics3DObject zLinkLinkGraphics = new Graphics3DObject();
      zLinkLinkGraphics.addSphere(radius, YoAppearance.Gray());
      zLink.setLinkGraphics(zLinkLinkGraphics);
      zJoint.setLink(zLink);

      externalForcePoint = new ExternalForcePoint("ef_" + name, new Vector3D(), this.getRobotsYoVariableRegistry());
      zJoint.addExternalForcePoint(externalForcePoint);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicPosition contactPoint = new YoGraphicPosition(name + "Point", externalForcePoint.getYoPosition(), radius, YoAppearance.Red());
      YoGraphicVector pointMassForce = new YoGraphicVector(name + "Force", externalForcePoint.getYoPosition(), externalForcePoint.getYoForce(), forceVectorScale, YoAppearance.Red());

      yoGraphicsListRegistry.registerYoGraphic(name, contactPoint);
      yoGraphicsListRegistry.registerYoGraphic(name, pointMassForce);

      this.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      addRootJoint(xJoint);
      xJoint.addJoint(yJoint);
      yJoint.addJoint(zJoint);
   }
   
   public PointMassRobot(String name, double mass)
   {
	   this(name, mass, DEFAULT_RADIUS,DEFAULT_FORCE_VECTOR_SCALE);
   }

   public void setPosition(double x, double y, double z)
   {
      xJoint.setQ(x);
      yJoint.setQ(y);
      zJoint.setQ(z);
   }

   public void setPosition(Point3D point)
   {
      xJoint.setQ(point.getX());
      yJoint.setQ(point.getY());
      zJoint.setQ(point.getZ());

   }

   public void setVelocity(double xd, double yd, double zd)
   {
      xJoint.setQd(xd);
      yJoint.setQd(yd);
      zJoint.setQd(zd);
   }

   public void setVelocity(Vector3D velocity)
   {
      xJoint.setQd(velocity.getX());
      yJoint.setQd(velocity.getY());
      zJoint.setQd(velocity.getZ());

   }

   public ExternalForcePoint getExternalForcePoint()
   {
      return externalForcePoint;
   }

   public void getPosition(Tuple3DBasics tuple3d)
   {
      double x = xJoint.getQYoVariable().getDoubleValue();
      double y = yJoint.getQYoVariable().getDoubleValue();
      double z = zJoint.getQYoVariable().getDoubleValue();

      tuple3d.set(x, y, z);
   }

   public void getVelocity(Tuple3DBasics tuple3d)
   {
      double xd = xJoint.getQDYoVariable().getDoubleValue();
      double yd = yJoint.getQDYoVariable().getDoubleValue();
      double zd = zJoint.getQDYoVariable().getDoubleValue();

      tuple3d.set(xd, yd, zd);
   }

   public void setMass(double mass)
   {
      zLink.setMass(mass);
   }


}
