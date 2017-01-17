package us.ihmc.simulationconstructionset.examples;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.WavyGroundProfile;

public class FallingBrickRobot extends Robot implements RobotController
{
   private static final long serialVersionUID = 773713164696806099L;
  
   private static final double BASE_H = 0.1, BASE_W = 0.2, BASE_L = 0.3;
   private static final double B1 = BASE_H / 2.0;
   private static final double M1 = 1.7;
   private static final double Ixx1 = 0.1, Iyy1 = 0.5, Izz1 = 0.9;
   private static final double G = 9.81;

   private final YoVariableRegistry registry = new YoVariableRegistry("FallingBrickController");

   // position, velocity, and acceleration variables
   DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z, qdd_x, qdd_y, qdd_z;
   DoubleYoVariable q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz, qdd_wx, qdd_wy, qdd_wz;

   DoubleYoVariable energy, q_qlength, theta_x;
   DoubleYoVariable qdd2_wx, qdd2_wy, qdd2_wz;

   Joint floatingJoint;


   public FallingBrickRobot()
   {
      super("FallingBrick");
      
      this.setGravity(0.0, 0.0, -G);

      // create the brick as a floating joint
      floatingJoint = new FloatingJoint("base", new Vector3d(0.0, 0.0, 0.0), this);
      Link link1 = base("base", YoAppearance.Red());
      floatingJoint.setLink(link1);
      this.addRootJoint(floatingJoint);

      // add ground contact points to the brick
      GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3d(BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc1);
      GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3d(BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc2);
      GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3d(-BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc3);
      GroundContactPoint gc4 = new GroundContactPoint("gc4", new Vector3d(-BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc4);

      GroundContactPoint gc5 = new GroundContactPoint("gc5", new Vector3d(BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc5);
      GroundContactPoint gc6 = new GroundContactPoint("gc6", new Vector3d(BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc6);
      GroundContactPoint gc7 = new GroundContactPoint("gc7", new Vector3d(-BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc7);
      GroundContactPoint gc8 = new GroundContactPoint("gc8", new Vector3d(-BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc8);

      GroundContactPoint gc9 = new GroundContactPoint("gc9", new Vector3d(0.0, 0.0, BASE_H / 2.0 + BASE_H), this);
      floatingJoint.addGroundContactPoint(gc9);
      GroundContactPoint gc10 = new GroundContactPoint("gc10", new Vector3d(0.0, 0.0, -BASE_H / 2.0 - BASE_H), this);
      floatingJoint.addGroundContactPoint(gc10);

      this.setController(this); // tells the simulator to call the local doControl() method 

      // instantiate ground contact model 
      GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
      // GroundContactModel groundModel = new CollisionGroundContactModel(this, 0.5, 0.7);

      GroundProfile3D profile = new WavyGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);

      initRobot();
      initControl();
   }

   /**
    * This method returns a brick link instance.
    */
   private Link base(String name, AppearanceDefinition appearance)
   {
      Link ret = new Link(name);
      ret.setMass(M1);
      ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -B1);

      // linkGraphics.addCube((float)BASE_L, (float)BASE_W, (float)BASE_H, appearance);
      // linkGraphics.addCone((float)BASE_L,(float)BASE_W);
      linkGraphics.addPyramidCube(BASE_L, BASE_W, BASE_H, BASE_H, appearance);

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   /**
    * This method sets the initial positions, velocities, and accelerations of the brick
    */
   public void initRobot()
   {
      q_qlength = new DoubleYoVariable("q_qlength", registry);
      theta_x = new DoubleYoVariable("theta_x", registry);

      t.set(0.0);

      q_x = (DoubleYoVariable)this.getVariable("q_x");
      q_y = (DoubleYoVariable)this.getVariable("q_y");
      q_z = (DoubleYoVariable)this.getVariable("q_z");
      qd_x = (DoubleYoVariable)this.getVariable("qd_x");
      qd_y = (DoubleYoVariable)this.getVariable("qd_y");
      qd_z = (DoubleYoVariable)this.getVariable("qd_z");
      qdd_x = (DoubleYoVariable)this.getVariable("qdd_x");
      qdd_y = (DoubleYoVariable)this.getVariable("qdd_y");
      qdd_z = (DoubleYoVariable)this.getVariable("qdd_z");

      q_qs = (DoubleYoVariable)this.getVariable("q_qs");
      q_qx = (DoubleYoVariable)this.getVariable("q_qx");
      q_qy = (DoubleYoVariable)this.getVariable("q_qy");
      q_qz = (DoubleYoVariable)this.getVariable("q_qz");
      qd_wx = (DoubleYoVariable)this.getVariable("qd_wx");
      qd_wy = (DoubleYoVariable)this.getVariable("qd_wy");
      qd_wz = (DoubleYoVariable)this.getVariable("qd_wz");
      qdd_wx = (DoubleYoVariable)this.getVariable("qdd_wx");
      qdd_wy = (DoubleYoVariable)this.getVariable("qdd_wy");
      qdd_wz = (DoubleYoVariable)this.getVariable("qdd_wz");

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
      qdd2_wx = new DoubleYoVariable("qdd2_wx", registry);
      qdd2_wy = new DoubleYoVariable("qdd2_wy", registry);
      qdd2_wz = new DoubleYoVariable("qdd2_wz", registry);

      energy = new DoubleYoVariable("energy", registry);
   }

   public void doControl()
   {
      energy.set(M1 * G * q_z.getDoubleValue() + 0.5 * M1 * qd_x.getDoubleValue() * qd_x.getDoubleValue() + 0.5 * M1 * qd_y.getDoubleValue() * qd_y.getDoubleValue() + 0.5 * M1 * qd_z.getDoubleValue() * qd_z.getDoubleValue()
                   + 0.5 * Ixx1 * qd_wx.getDoubleValue() * qd_wx.getDoubleValue() + 0.5 * Iyy1 * qd_wy.getDoubleValue() * qd_wy.getDoubleValue() + 0.5 * Izz1 * qd_wz.getDoubleValue() * qd_wz.getDoubleValue());

      qdd2_wx.set((Iyy1 - Izz1) / Ixx1 * qd_wy.getDoubleValue() * qd_wz.getDoubleValue());
      qdd2_wy.set((Izz1 - Ixx1) / Iyy1 * qd_wz.getDoubleValue() * qd_wx.getDoubleValue());
      qdd2_wz.set((Ixx1 - Iyy1) / Izz1 * qd_wx.getDoubleValue() * qd_wy.getDoubleValue());

   }

   public YoVariableRegistry getYoVariableRegistry()
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

