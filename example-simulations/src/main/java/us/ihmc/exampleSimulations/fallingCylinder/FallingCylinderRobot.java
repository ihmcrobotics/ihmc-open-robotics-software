package us.ihmc.exampleSimulations.fallingCylinder;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FallingCylinderRobot extends Robot
{
   private static final long serialVersionUID = 773713164696806099L;

   private static final double BASE_H = 0.1, BASE_W = 0.2, BASE_L = 0.3;
   private static final double B1 = BASE_H / 2.0;
   private static final double M1 = 1.7;
   private static final double Ixx1 = 0.1, Iyy1 = 0.5, Izz1 = 0.9;
   private static final double G = 9.81;
   private final YoRegistry registry = new YoRegistry("FallingCylinderController");

   // position, velocity, and acceleration variables
   YoDouble q_x, q_y, q_z, qd_x, qd_y, qd_z, qdd_x, qdd_y, qdd_z;
   YoDouble q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz, qdd_wx, qdd_wy, qdd_wz;
   YoDouble energy, q_qlength, theta_x;
   YoDouble qdd2_wx, qdd2_wy, qdd2_wz;
   Joint floatingJoint;

   public FallingCylinderRobot()
   {
      super("FallingCylinder");

      this.setGravity(0.0, 0.0, -G);

      // create the Cylinder as a floating joint
      floatingJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);
      Link link1 = base("base", YoAppearance.Red());
      floatingJoint.setLink(link1);
      this.addRootJoint(floatingJoint);

      // add ground contact points to the cylinder
      GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3D(BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc1);
      GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3D(BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc2);
      GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3D(-BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc3);
      GroundContactPoint gc4 = new GroundContactPoint("gc4", new Vector3D(-BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc4);
      GroundContactPoint gc5 = new GroundContactPoint("gc5", new Vector3D(BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc5);
      GroundContactPoint gc6 = new GroundContactPoint("gc6", new Vector3D(BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc6);
      GroundContactPoint gc7 = new GroundContactPoint("gc7", new Vector3D(-BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc7);
      GroundContactPoint gc8 = new GroundContactPoint("gc8", new Vector3D(-BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc8);
      GroundContactPoint gc9 = new GroundContactPoint("gc9", new Vector3D(0.0, 0.0, BASE_H / 2.0 + BASE_H), this);
      floatingJoint.addGroundContactPoint(gc9);
      GroundContactPoint gc10 = new GroundContactPoint("gc10", new Vector3D(0.0, 0.0, -BASE_H / 2.0 - BASE_H), this);
      floatingJoint.addGroundContactPoint(gc10);

      
      //   instantiate ground contact model
      GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoRegistry()); // TODO find the meaning

      GroundProfile3D profile = new WavyGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);
      initRobot();
      initControl();
   }

   /*
    * This method returns a Cylinder link instance
    */
   private Link base(String name, AppearanceDefinition appearance)
   {
      Link ret = new Link(name);
      ret.setMass(M1);
      ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);
      ret.setComOffset(0.0, 0.0, 0.0);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -B1);
      linkGraphics.addPyramidCube(BASE_L, BASE_W, BASE_H, BASE_H, appearance);

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   /*
    * This method sets the initial positions, velocities, and accelerations of the brick
    */
   public void initRobot()
   {
      q_qlength = new YoDouble("q_qlength", registry);
      theta_x = new YoDouble("theta_x", registry);
      t.set(0.0);
      q_x = (YoDouble) this.findVariable("q_x");
      q_y = (YoDouble) this.findVariable("q_y");
      q_z = (YoDouble) this.findVariable("q_z");
      qd_x = (YoDouble) this.findVariable("qd_x");
      qd_y = (YoDouble) this.findVariable("qd_y");
      qd_z = (YoDouble) this.findVariable("qd_z");
      qdd_x = (YoDouble) this.findVariable("qdd_x");
      qdd_y = (YoDouble) this.findVariable("qdd_y");
      qdd_z = (YoDouble) this.findVariable("qdd_z");
      q_qs = (YoDouble) this.findVariable("q_qs");
      q_qx = (YoDouble) this.findVariable("q_qx");
      q_qy = (YoDouble) this.findVariable("q_qy");
      q_qz = (YoDouble) this.findVariable("q_qz");
      qd_wx = (YoDouble) this.findVariable("qd_wx");
      qd_wy = (YoDouble) this.findVariable("qd_wy");
      qd_wz = (YoDouble) this.findVariable("qd_wz");
      qdd_wx = (YoDouble) this.findVariable("qdd_wx");
      qdd_wy = (YoDouble) this.findVariable("qdd_wy");
      qdd_wz = (YoDouble) this.findVariable("qdd_wz");
      q_x.set(0.0);
      q_y.set(0.0);
      q_z.set(0.6);
      qd_x.set(0.0);
      qd_y.set(0.0);
      qd_z.set(0.0);
      q_qs.set(0.707);
      q_qx.set(0.3);
      q_qy.set(0.4);
      q_qz.set(0.5);
      qd_wx.set(0.0001);
      qd_wy.set(1.0);
      qd_wz.set(0.5001);
   }

   public void initControl()
   {
      qdd2_wx = new YoDouble("qdd2_wx", registry);
      qdd2_wy = new YoDouble("qdd2_wy", registry);
      qdd2_wz = new YoDouble("qdd2_wz", registry);
      energy = new YoDouble("energy", registry);
   }

   public void doControl()
   {
      energy.set(M1 * G * q_z.getDoubleValue() + 0.5 * M1 * qd_x.getDoubleValue() * qd_x.getDoubleValue()
            + 0.5 * M1 * qd_y.getDoubleValue() * qd_y.getDoubleValue() + 0.5 * M1 * qd_z.getDoubleValue() * qd_z.getDoubleValue()
            + 0.5 * Ixx1 * qd_wx.getDoubleValue() * qd_wx.getDoubleValue() + 0.5 * Iyy1 * qd_wy.getDoubleValue() * qd_wy.getDoubleValue()
            + 0.5 * Izz1 * qd_wz.getDoubleValue() * qd_wz.getDoubleValue());

      qdd2_wx.set((Iyy1 - Izz1) / Ixx1 * qd_wy.getDoubleValue() * qd_wz.getDoubleValue());
      qdd2_wy.set((Izz1 - Ixx1) / Iyy1 * qd_wz.getDoubleValue() * qd_wx.getDoubleValue());
      qdd2_wz.set((Ixx1 - Iyy1) / Izz1 * qd_wx.getDoubleValue() * qd_wy.getDoubleValue());
   }
   
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
   public void initialize()
   {
   }
   public String getDescription()
   {
       return getName();
   }

}
