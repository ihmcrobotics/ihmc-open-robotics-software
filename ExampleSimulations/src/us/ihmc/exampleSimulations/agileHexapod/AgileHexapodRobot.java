package us.ihmc.exampleSimulations.agileHexapod;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.CollisionGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;

//February 17, 1996: Ann Torres and Jerry Pratt.  Agile Hexapod created using MIT Leg Laboratory Creature Library.
//June 8, 2001:  Jerry Pratt.  Ported to Yobotics! Simulation Construction Set.

public class AgileHexapodRobot extends Robot
{
// private static final double
//    K_HIP_STOP = 15.0, B_HIP_STOP = 2.0, K_KNEE_STOP = 15.0, B_KNEE_STOP = 2.0;

   /* size of the body, thigh, & shin */
   /* x,y,z lengths when angles=0 */

   private static final double
      BODY_MASS = 3.087705, BODY_Ixx = 0.006519, BODY_Iyy = 0.038129, BODY_Izz = 0.040696;

// private static final double
//    BODY_COM_X = -0.007312, BODY_COM_Y = 0.0, BODY_COM_Z = 0.001214;

   private static final double
      PIN_MASS = 0.001696, PIN_Ixx = 0.0, PIN_Iyy = 0.0, PIN_Izz = 0.0;
   private static final double
      PIN_COM_X = 0.0, PIN_COM_Y = 0.0, PIN_COM_Z = 0.0;

   private static final double
      THIGH_MASS = 0.086552, THIGH_Ixx = 0.000161, THIGH_Iyy = 0.000161, THIGH_Izz = 0.000020;
   private static final double
      THIGH_COM_X = 0.0, THIGH_COM_Y = 0.0, THIGH_COM_Z = -0.050000;

   private static final double
      SHIN_MASS = 0.039566, SHIN_Ixx = 0.000033, SHIN_Iyy = 0.000033, SHIN_Izz = 0.000008;
   private static final double
      SHIN_COM_X = 0.0, SHIN_COM_Y = 0.0, SHIN_COM_Z = -0.046786;

   private static final double
      ROD_MASS = 0.182862, ROD_Ixx = 0.000175, ROD_Iyy = 0.000175, ROD_Izz = 0.000116;
   private static final double
      ROD_COM_X = 0.0, ROD_COM_Y = 0.0, ROD_COM_Z = 0.298370;

   public static final double BODY_X = 0.17;
   private static final double BODY_XB = 0.10;
   private static final double BODY_XF = 0.17;
   private static final double BODY_Y = 0.08;
   private static final double BODY_Z = 0.05;


   private static final double BAND_X = (0.10 + .075);
   private static final double BAND_Y = 0.08;
   private static final double BAND_Z = 0.05;

   private static final double BODY_TOT = (BODY_XB + BODY_XF + BAND_X);
   private static final double BODY_OFF = ((BODY_TOT / 2.0) - BODY_XB - (BAND_X / 2.0));

   private static final double TENNA_LEN = BODY_TOT;
   private static final double TENNA_RAD = (TENNA_LEN / 40.0);

// private static final double THIGH_X = 0.04;
// private static final double THIGH_Y = 0.04;
   private static final double THIGH_Z = 0.10;
   private static final double THIGH_RAD = 0.02;

// private static final double SHIN_X = 0.04;
// private static final double SHIN_Y = 0.04;
   private static final double SHIN_Z = 0.08;    /* lower leg */
   private static final double SHIN_RAD = 0.02;    /* foot radius */

   private static final double SPONGE_Z = 0.005;
   private static final double CONTACT_Z = (SHIN_Z + SHIN_RAD - SPONGE_Z);

   private static final double JOINT_RAD = 0.025;


   /* position of each leg/hip wrt {B} */

   /* first set of legs */
   public static final double PX1 = (0.00);
   public static final double PY1 = (-0.075);
   public static final double PZ1 = (0.00);

   public static final double PX2 = (0.10 + .050);
   public static final double PY2 = (0.075);
   public static final double PZ2 = (0.00);

   public static final double PX3 = (-0.10 - .050);
   public static final double PY3 = (0.075);
   public static final double PZ3 = (0.00);


   /* second set of legs */
   public static final double PX4 = (0.00);
   public static final double PY4 = (0.075);
   public static final double PZ4 = (0.00);

   public static final double PX5 = (0.10 + .050);
   public static final double PY5 = (-0.075);
   public static final double PZ5 = (0.00);

   public static final double PX6 = (-0.10 - .050);
   public static final double PY6 = (-0.075);
   public static final double PZ6 = (0.00);

   /* pendulum dimensions */
   public static final double PEND_L = 0.30;
   private static final double ROD_R = 0.005;
   private static final double PEND_R = 0.04;

   public static final double LEN = THIGH_Z;

   private FloatingJoint floatingJoint;

   DoubleYoVariable gc_lheel_x, gc_lheel_z, gc_lheel_dx, gc_lheel_dz, gc_lheel_fs, gc_lheel_tdx, gc_lheel_tdz, gc_lheel_fx, gc_lheel_fz;
   private static final int
      LEFT = 0, RIGHT = 1;

   public AgileHexapodRobot()
   {
      super("AgileHexapod");
      this.setGravity(0.0, 0.0, -9.81);

      // Body:
      floatingJoint = new FloatingJoint("floating", new Vector3d(), this);
      Link body = body();
      floatingJoint.setLink(body);
      this.addRootJoint(floatingJoint);

      GroundContactPoint bodyContact1 = new GroundContactPoint("gc_body1", new Vector3d(BODY_TOT * 0.35, -BODY_Y / 2.0, -BODY_Z / 2.0), this);
      GroundContactPoint bodyContact2 = new GroundContactPoint("gc_body2", new Vector3d(BODY_TOT * 0.35, BODY_Y / 2.0, -BODY_Z / 2.0), this);
      GroundContactPoint bodyContact3 = new GroundContactPoint("gc_body3", new Vector3d(-BODY_TOT * 0.35, BODY_Y / 2.0, -BODY_Z / 2.0), this);
      GroundContactPoint bodyContact4 = new GroundContactPoint("gc_body4", new Vector3d(-BODY_TOT * 0.35, -BODY_Y / 2.0, -BODY_Z / 2.0), this);

      floatingJoint.addGroundContactPoint(bodyContact1);
      floatingJoint.addGroundContactPoint(bodyContact2);
      floatingJoint.addGroundContactPoint(bodyContact3);
      floatingJoint.addGroundContactPoint(bodyContact4);

      // Legs:
      buildLeg(RIGHT, "hip1_z", "hip1_x", "knee1", "gc_foot1", new Vector3d(PX1, PY1, PZ1));
      buildLeg(LEFT, "hip2_z", "hip2_x", "knee2", "gc_foot2", new Vector3d(PX2, PY2, PZ2));
      buildLeg(LEFT, "hip3_z", "hip3_x", "knee3", "gc_foot3", new Vector3d(PX3, PY3, PZ3));
      buildLeg(LEFT, "hip4_z", "hip4_x", "knee4", "gc_foot4", new Vector3d(PX4, PY4, PZ4));
      buildLeg(RIGHT, "hip5_z", "hip5_x", "knee5", "gc_foot5", new Vector3d(PX5, PY5, PZ5));
      buildLeg(RIGHT, "hip6_z", "hip6_x", "knee6", "gc_foot6", new Vector3d(PX6, PY6, PZ6));


      // Pendulum:
      PinJoint pend1 = new PinJoint("pend1", new Vector3d(), this, Axis.Y);
      Link pinLink = pinLink();
      pend1.setLink(pinLink);
      floatingJoint.addJoint(pend1);

      PinJoint pend2 = new PinJoint("pend2", new Vector3d(), this, Axis.X);
      Link rodLink = rodLink();
      pend2.setLink(rodLink);
      pend1.addJoint(pend2);

      GroundContactPoint pend_end = new GroundContactPoint("gc_pend_end", new Vector3d(0.0, 0.0, PEND_L), this);
      pend2.addGroundContactPoint(pend_end);

      // Set controller and ground contact model:
      
      YoVariableRegistry groundRegistry = new YoVariableRegistry("Ground");
      
      // GroundContactModel groundModel = new LinearGroundContactModel(this, 2000.0, 100.0, 50.0, 50.0, groundRegistry);
      GroundContactModel groundModel = new CollisionGroundContactModel(this, 0.2, 0.7, groundRegistry);
      this.addYoVariableRegistry(groundRegistry);
      
      GroundProfile3D profile = null;

      double xMin = -20.0, xMax = 20.0, yMin = -20.0, yMax = 20.0;
      double amplitude = 0.1, frequency = 0.3, offset = 0.0;

      double x_amp1 = 0.04, x_freq1 = 0.4, x_amp2 = 0.1, x_freq2 = 0.2, y_amp1 = 0.02, y_freq1 = 0.35, y_amp2 = 0.0, y_freq2 = 5.0;

      // double x_amp1 = 0.20, x_freq1 = 0.2, x_amp2 = 0.0, x_freq2 = 2.3,
      // y_amp1 = 0.1, y_freq1 = 0.2, y_amp2 = 0.0, y_freq2 = 5.0;

      int ROLLING = 0, BUMPY = 1;
      int GROUND = ROLLING;    // BUMPY; //

      if (GROUND == ROLLING)
      {
         profile = new RollingGroundProfile(amplitude, frequency, offset, xMin, xMax, yMin, yMax);
      }

      else if (GROUND == BUMPY)
      {
         profile = new BumpyGroundProfile(x_amp1, x_freq1, x_amp2, x_freq2, y_amp1, y_freq1, y_amp2, y_freq2, xMin, xMax, yMin, yMax);
      }


      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);

      this.setController(new AgileHexapodController(this, profile.getHeightMapIfAvailable(), "agileHexapodController"));
   }

   private void buildLeg(int side, String hipzName, String hipxName, String kneeName, String footName, Vector3d legOffset)
   {
      /* Building Leg */
      PinJoint hip_z = new PinJoint(hipzName, legOffset, this, Axis.Z);

      // hip_z.setLimitStops(-0.9*Math.PI/2.0,0.9*Math.PI/2.0,K_HIP_STOP, B_HIP_STOP);
      hip_z.setDamping(0.005);
      Link pinLink = pinLink();
      hip_z.setLink(pinLink);
      floatingJoint.addJoint(hip_z);

      PinJoint hip_x = new PinJoint(hipxName, new Vector3d(), this, Axis.X);
      Link thighLink = thighLink();
      hip_x.setLink(thighLink);
      hip_x.setDamping(0.005);
      hip_z.addJoint(hip_x);

      PinJoint knee = new PinJoint(kneeName, new Vector3d(0.0, 0.0, -THIGH_Z), this, Axis.X);

      // if (side == LEFT) knee.setLimitStops(0.0,0.9*Math.PI, K_KNEE_STOP, B_KNEE_STOP);
      // else knee.setLimitStops(-0.9*Math.PI,0.0, K_KNEE_STOP, B_KNEE_STOP);
      knee.setDamping(0.005);
      Link shinLink;
      shinLink = shinLink();
      knee.setLink(shinLink);
      hip_x.addJoint(knee);

      GroundContactPoint foot = new GroundContactPoint(footName, new Vector3d(0.0, 0.0, -CONTACT_Z), this);
      knee.addGroundContactPoint(foot);

   }

   private Link body()
   {
      Link ret = new Link("body");

      ret.setMass(BODY_MASS);
      ret.setMomentOfInertia(BODY_Ixx, BODY_Iyy, BODY_Izz);

      // ret.setComOffset(BODY_COM_X, BODY_COM_Y, BODY_COM_Z);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(-BAND_X / 2.0, 0.0, 0.0);
      linkGraphics.translate(-BODY_OFF, 0.0, 0.0);
      linkGraphics.rotate(-Math.PI / 2.0, Axis.Y);
      linkGraphics.addHemiEllipsoid(BODY_Z, BODY_Y, BODY_XB, YoAppearance.PlaneMaterial());

      linkGraphics.identity();
      linkGraphics.translate(BAND_X / 2.0, 0.0, 0.0);
      linkGraphics.translate(-BODY_OFF, 0.0, 0.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.addHemiEllipsoid(BODY_Z, BODY_Y, BODY_XF, YoAppearance.PlaneMaterial());

      linkGraphics.identity();
      linkGraphics.translate(-BAND_X / 2.0, 0.0, 0.0);
      linkGraphics.translate(-BODY_OFF, 0.0, 0.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);

      // use_color("wood_material");
      linkGraphics.addGenTruncatedCone(BAND_X, BAND_Z, BAND_Y, BAND_Z, BAND_Y, YoAppearance.DarkRed());    // WoodMaterial

      linkGraphics.identity();
      linkGraphics.translate(BODY_TOT * 0.45, BODY_Y / 4.0, 0.0);
      linkGraphics.rotate(-Math.PI / 3.0, Axis.Y);
      linkGraphics.rotate(-Math.PI / 6.0, Axis.X);
      linkGraphics.addCone(TENNA_LEN, TENNA_RAD, YoAppearance.BlackMetalMaterial());

      linkGraphics.identity();
      linkGraphics.translate(BODY_TOT * 0.45, -BODY_Y / 4.0, 0.0);
      linkGraphics.rotate(-Math.PI / 3.0, Axis.Y);
      linkGraphics.rotate(Math.PI / 6.0, Axis.X);
      linkGraphics.addCone(TENNA_LEN, TENNA_RAD, YoAppearance.BlackMetalMaterial());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }




   private Link pinLink()
   {
      Link ret = new Link("pin");

      ret.setMass(PIN_MASS);
      ret.setMomentOfInertia(PIN_Ixx, PIN_Iyy, PIN_Izz);
      ret.setComOffset(PIN_COM_X, PIN_COM_Y, PIN_COM_Z);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -THIGH_Z / 100.0);
      linkGraphics.addCylinder(THIGH_Z / 50.0, THIGH_RAD);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }



   private Link thighLink()
   {
      Link ret = new Link("thigh");

      ret.setMass(THIGH_MASS);
      ret.setMomentOfInertia(THIGH_Ixx, THIGH_Iyy, THIGH_Izz);
      ret.setComOffset(THIGH_COM_X, THIGH_COM_Y, THIGH_COM_Z);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(JOINT_RAD, YoAppearance.BlackMetalMaterial());
      linkGraphics.translate(0.0, 0.0, -THIGH_Z);
      linkGraphics.addCylinder(THIGH_Z, THIGH_RAD);
      linkGraphics.addSphere(JOINT_RAD, YoAppearance.BlackMetalMaterial());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link shinLink()
   {
      Link ret = new Link("shin");

      ret.setMass(SHIN_MASS);
      ret.setMomentOfInertia(SHIN_Ixx, SHIN_Iyy, SHIN_Izz);
      ret.setComOffset(SHIN_COM_X, SHIN_COM_Y, SHIN_COM_Z);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -SHIN_Z);
      linkGraphics.addCylinder(SHIN_Z, SHIN_RAD, YoAppearance.BlackMetalMaterial());
      linkGraphics.identity();
      linkGraphics.translate(0.0, 0.0, -SHIN_Z);
      linkGraphics.rotate(Math.PI, Axis.Y);
      linkGraphics.addHemiEllipsoid(SHIN_RAD, SHIN_RAD, SHIN_RAD, YoAppearance.BlackMetalMaterial());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link rodLink()
   {
      Link ret = new Link("rod");

      ret.setMass(ROD_MASS);
      ret.setMomentOfInertia(ROD_Ixx, ROD_Iyy, ROD_Izz);
      ret.setComOffset(ROD_COM_X, ROD_COM_Y, ROD_COM_Z);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(PEND_L, ROD_R, YoAppearance.BlackMetalMaterial());
      linkGraphics.translate(0.0, 0.0, PEND_L);
      linkGraphics.addSphere(PEND_R, YoAppearance.DarkRed());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }




}
