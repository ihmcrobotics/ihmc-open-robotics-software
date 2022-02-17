package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.FixedJoint;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoVariables.variable.YoDouble;

public class SinggleLeggedRobot extends Robot
{
   // Parameters of the robot.
   public static final double COORD_LENGTH = 0.5;

   public static final double BASE_W = 0.025, BASE_D = 0.025, BASE_H = 1.0;
   public static final double BASE_MASS = 1000000, BASE_Ixx = 1.0, BASE_Iyy = 1.0, BASE_Izz = 1.0;

   public static final double HIP_H = 0.025, HIP_R = 0.05;
   public static final double HIP_MASS = 3.0, HIP_Ixx = 0.00203125, HIP_Iyy = 0.00375000, HIP_Izz = 0.00203125;

   public static final double UPPER_LEG_W = 0.02, UPPER_LEG_D = 0.03, UPPER_LEG_H = 0.3;
   public static final double UPPER_LEG_MASS = 1.0, UPPER_LEG_Ixx = 1.0, UPPER_LEG_Iyy = 1.0, UPPER_LEG_Izz = 1.0;

   public static final double LOWER_LEG_W = 0.02, LOWER_LEG_D = 0.03, LOWER_LEG_H = 0.3;
   public static final double LOWER_LEG_MASS = 1.0, LOWER_LEG_Ixx = 1.0, LOWER_LEG_Iyy = 1.0, LOWER_LEG_Izz = 1.0;
   public static final double FOOT_R = 0.01;

   // Joints.
   //   private final FloatingPlanarJoint plane;
   private final PinJoint plane;
   private final SliderJoint hipSliderJoint;
   private final PinJoint hipPinJoint;
   private final PinJoint kneePinJoint;
   private final IMUMount imuMount;
   private final RigidBodyTransform imuTransform;

   // Initial state of Robot
   private double initialHipPosition = 45 * Math.PI / 180.0;
   private double initialKneePosition = -90 * Math.PI / 180.0;

   // States of the Robot
   private YoDouble t, q_plane, q_hip_slide, q_hip_rev, q_knee_rev;
   private YoDouble qd_hip_slide, qd_hip_rev, qd_knee_rev;
   private YoDouble qdd_hip_slide, qdd_hip_rev, qdd_knee_rev;
   private YoDouble tau_hip, tau_knee;

   private GroundContactPoint gc_base;
   private GroundContactPoint gc_foot;

   public SinggleLeggedRobot()
   {
      super("singgleLeggedRobot");

      // Construct the robot

      plane = new PinJoint("plane", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.X);
      plane.setLink(baseLink());
      this.addRootJoint(plane);

      hipSliderJoint = new SliderJoint("hip_slide", new Vector3D(0.0, -BASE_D, 0.425), this, Axis3D.Z);
      hipSliderJoint.setLink(hipLink());
      plane.addJoint(hipSliderJoint);

      hipPinJoint = new PinJoint("hip_rev", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.Y);
      hipPinJoint.setLink(upperLegLink());
      hipSliderJoint.addJoint(hipPinJoint);

      imuTransform = new RigidBodyTransform();
      imuTransform.getTranslation().set(new Vector3D(0.0, 0.0, 0.0));
      imuMount = new IMUMount("SinggleLeggedIMU", imuTransform, this);
      hipPinJoint.addIMUMount(imuMount);

      kneePinJoint = new PinJoint("knee_rev", new Vector3D(0.0, 0.0, -UPPER_LEG_H), this, Axis3D.Y);
      kneePinJoint.setLink(lowerLegLink());
      hipPinJoint.addJoint(kneePinJoint);

      // Extract YoVariables
      t = (YoDouble) this.findVariable("t");
      q_plane = plane.getQYoVariable();
      q_hip_slide = hipSliderJoint.getQYoVariable();
      qd_hip_slide = hipSliderJoint.getQDYoVariable();
      qdd_hip_slide = hipSliderJoint.getQDDYoVariable();

      //      hipPinJoint.getTransformToWorld(imuTransform);
      q_hip_rev = hipPinJoint.getQYoVariable();
      qd_hip_rev = hipPinJoint.getQDYoVariable();
      qdd_hip_rev = hipPinJoint.getQDDYoVariable();

      tau_hip = hipPinJoint.getTauYoVariable();
      q_knee_rev = kneePinJoint.getQYoVariable();
      qd_knee_rev = kneePinJoint.getQDYoVariable();
      qdd_knee_rev = kneePinJoint.getQDDYoVariable();
      tau_knee = kneePinJoint.getTauYoVariable();

      // add ground contact point
      gc_base = new GroundContactPoint("gc_base", new Vector3D(0.0, 0.0, 0.0), this);
      plane.addGroundContactPoint(gc_base);
      gc_foot = new GroundContactPoint("gc_foot", new Vector3D(0.0, 0.0, -(LOWER_LEG_H + FOOT_R)), this);
      kneePinJoint.addGroundContactPoint(gc_foot);

      // instantiate ground contact model
      //      GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoRegistry());
      GroundContactModel groundModel = new LinearGroundContactModel(this, 14220, 150.6, 125.0, 300.0, this.getRobotsYoRegistry());
      this.setGroundContactModel(groundModel);

      initRobot();
   }

   private Link baseLink()
   {
      Link baseLink = new Link("BaseLink");
      baseLink.setMass(BASE_MASS);
      baseLink.setMomentOfInertia(BASE_Ixx, BASE_Iyy, BASE_Izz);

      Graphics3DObject baseGraphics = new Graphics3DObject();
      baseGraphics.addCoordinateSystem(COORD_LENGTH);
      baseGraphics.addCube(BASE_W, BASE_D, BASE_H);
      baseLink.setLinkGraphics(baseGraphics);

      return baseLink;
   }

   private Link hipLink()
   {
      Link hipLink = new Link("HipLink");
      hipLink.setMass(HIP_MASS);
      hipLink.setMomentOfInertia(HIP_Ixx, HIP_Iyy, HIP_Izz);

      Graphics3DObject hipGraphics = new Graphics3DObject();
      hipGraphics.translate(0.0, 0.0, 0.0);
      hipGraphics.rotate(0.5 * Math.PI, Axis3D.X);
      //            hipGraphics.addCoordinateSystem(COORD_LENGTH);
      hipGraphics.addCylinder(HIP_H, HIP_R, YoAppearance.Aquamarine());
      hipGraphics.addCube(0.01, 2 * HIP_R, 0.03, YoAppearance.BlueViolet());

      hipLink.setLinkGraphics(hipGraphics);

      return hipLink;
   }

   private Link upperLegLink()
   {
      Link upperLegLink = new Link("UpperLegLink");
      upperLegLink.setMass(UPPER_LEG_MASS);
      upperLegLink.setMomentOfInertia(UPPER_LEG_Ixx, UPPER_LEG_Iyy, UPPER_LEG_Izz);

      Graphics3DObject upperLegGraphics = new Graphics3DObject();
      upperLegGraphics.translate(0.0, 0.0, 0.0);
      upperLegGraphics.rotate(-Math.PI, Axis3D.Y);
      upperLegGraphics.addCoordinateSystem(COORD_LENGTH);
      upperLegGraphics.addCube(UPPER_LEG_W, UPPER_LEG_D, UPPER_LEG_H, YoAppearance.Aqua());

      upperLegLink.setLinkGraphics(upperLegGraphics);

      return upperLegLink;
   }

   private Link lowerLegLink()
   {
      Link lowerLegLink = new Link("LowerLegLink");
      lowerLegLink.setMass(LOWER_LEG_MASS);
      lowerLegLink.setMomentOfInertia(LOWER_LEG_Ixx, LOWER_LEG_Iyy, LOWER_LEG_Izz);

      Graphics3DObject lowerLegGraphics = new Graphics3DObject();
      lowerLegGraphics.translate(0.0, 0.0, 0.0); // should be changed
      lowerLegGraphics.rotate(-Math.PI, Axis3D.Y); // should be changed
      lowerLegGraphics.addCoordinateSystem(COORD_LENGTH);
      lowerLegGraphics.addCube(UPPER_LEG_W, UPPER_LEG_D, UPPER_LEG_H, YoAppearance.Aqua());

      lowerLegGraphics.translate(0.0, 0.0, UPPER_LEG_H);
      lowerLegGraphics.addSphere(FOOT_R, YoAppearance.AliceBlue());

      lowerLegLink.setLinkGraphics(lowerLegGraphics);

      return lowerLegLink;
   }

   public void initRobot()
   {
      hipPinJoint.setInitialState(initialHipPosition, 0.0);
      kneePinJoint.setInitialState(initialKneePosition, 0.0);
   }

   public double getT()
   {
      return t.getDoubleValue();
   }

   public void getIMUAngularVelocity(Vector3D imuAngularVelocity)
   {
      imuMount.getAngularVelocityInBody(imuAngularVelocity);
   }

   public double getHipSlidePosition()
   {
      return q_hip_slide.getDoubleValue();
   }

   public double getHipSlideVelocity()
   {
      return qd_hip_slide.getDoubleValue();
   }

   public double getHipSlideAcceleration()
   {
      return qdd_hip_slide.getDoubleValue();
   }

   public double getHipAngularPosition()
   {
      return q_hip_rev.getDoubleValue();
   }

   public double getHipAngularVelocity()
   {
      return qd_hip_rev.getDoubleValue();
   }

   public double getHipAngularAcceleration()
   {
      return qdd_hip_rev.getDoubleValue();
   }

   public double getKneeAngularPosition()
   {
      return q_knee_rev.getDoubleValue();
   }

   public double getKneeAngularVelocity()
   {
      return qd_knee_rev.getDoubleValue();
   }

   public double getKneeAngularAcceleration()
   {
      return qdd_knee_rev.getDoubleValue();
   }

   public double getGRFx()
   {
      return gc_foot.getYoForce().getX();
   }

   public double getGRFy()
   {
      return gc_foot.getYoForce().getY();
   }

   public double getGRFz()
   {
      return gc_foot.getYoForce().getZ();
   }

   public MatrixForML getGRF()
   {
      double x = getGRFx();
      double y = getGRFy();
      double z = getGRFz();

      MatrixForML ret = new MatrixForML(1, 3);
      ret.set(0, 0, x);
      ret.set(0, 1, y);
      ret.set(0, 2, z);

      return ret;
   }

   public double getHipTau()
   {
      return tau_hip.getDoubleValue();
   }

   public double getKneeTau()
   {
      return tau_knee.getDoubleValue();
   }

   public void setPlanePosition(double position)
   {
      q_plane.set(position);
   }

   public void setHipSlideVelocity(double velocity)
   {
      qd_hip_slide.set(velocity);
   }

   public void setHipSlideForce(double force)
   {
      hipSliderJoint.setTau(force);
   }

   public void setHipTorque(double hipTau)
   {
      if (hipTau < -50) // Hip joint torque limits
      {
         hipTau = -50.0;
      }
      else if (hipTau > 50)
      {
         hipTau = 50.0;
      }

      // In case which no contact in foot
      if (getGRFz() == 0.0)
      {
         hipTau = 0.0;
      }

      this.tau_hip.set(hipTau);
   }

   public void setKneeTorque(double kneeTau)
   {
      if (kneeTau < -50) // Knee joint torque limits
      {
         kneeTau = -50.0;
      }
      else if (kneeTau > 50)
      {
         kneeTau = 50.0;
      }

      // In case which no contact in foot
      if (getGRFz() == 0.0)
      {
         kneeTau = 0.0;
      }

      this.tau_knee.set(kneeTau);
   }

   public void setHipAngVelocity(double hipAngVel)
   {
      this.qd_hip_rev.set(hipAngVel);
   }

   public void setKneeAngVelocity(double kneeAngVel)
   {
      this.qd_knee_rev.set(kneeAngVel);
   }

   public void setToInitialState()
   {
      this.q_hip_slide.set(0.0);
      this.q_hip_rev.set(initialHipPosition);
      this.q_knee_rev.set(initialKneePosition);
      this.qd_hip_slide.set(0.0);
      this.qd_hip_rev.set(0.0);
      this.qd_knee_rev.set(0.0);
      this.qdd_hip_slide.set(0.0);
      this.qdd_hip_rev.set(0.0);
      this.qdd_knee_rev.set(0.0);
   }

}
