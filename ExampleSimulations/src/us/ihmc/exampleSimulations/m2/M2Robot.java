package us.ihmc.exampleSimulations.m2;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;

public class M2Robot extends Robot
{
   private final YoVariableRegistry registry = new YoVariableRegistry("Orientation");
   private final DoubleYoVariable q_yaw = new DoubleYoVariable("q_yaw", registry);
   private final  DoubleYoVariable q_roll = new DoubleYoVariable("q_roll", registry);
   private final  DoubleYoVariable q_pitch = new DoubleYoVariable("q_pitch", registry);

   protected FloatingJoint bodyJoint;
   private final M2Parameters m2Parameters;

   public M2Robot(M2Parameters m2Parameters)
   {
      super("M2");

      this.m2Parameters = m2Parameters;

      this.addYoVariableRegistry(registry);
      bodyJoint = new FloatingJoint("body", new Vector3D(), this);
      Link bodyLink = body();
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      // RIGHT LEG.

      Joint rightHipUni = new UniversalJoint("right_hip_yaw", "right_hip_roll", new Vector3D(0.0, -m2Parameters.HIP_OFFSET_Y.value, 0.0), this, Axis.Z,
                             Axis.X);
      Link rightWaistLink = waist();
      rightHipUni.setLink(rightWaistLink);
      bodyJoint.addJoint(rightHipUni);

      Joint rightHipPitch = new PinJoint("right_hip_pitch", new Vector3D(0.0, 0.0, -m2Parameters.HIP_JOINT_OFF.value), this, Axis.Y);
      Link rightThighLink = rightThigh();
      rightHipPitch.setLink(rightThighLink);
      rightHipUni.addJoint(rightHipPitch);

      PinJoint rightKnee = new PinJoint("right_knee", new Vector3D(0.0, -m2Parameters.HIP_TO_THIGH_OFF.value, -m2Parameters.THIGH_LENGTH.value), this, Axis.Y);
      Link rightShinLink = shin();
      rightKnee.setLink(rightShinLink);
      rightKnee.setLimitStops(0.0, Math.PI, 5000.0, 400.0);
      rightHipPitch.addJoint(rightKnee);

      Joint rightAnkleRoll = new PinJoint("right_ankle_roll", new Vector3D(0.0, 0.0, -m2Parameters.SHIN_LENGTH.value), this, Axis.X);
      Link rightRetinaculumLink = retinaculum();
      rightAnkleRoll.setLink(rightRetinaculumLink);
      rightKnee.addJoint(rightAnkleRoll);

      Joint rightAnklePitch = new PinJoint("right_ankle_pitch", new Vector3D(0.0, 0.0, -m2Parameters.ANKLE_JOINT_OFF.value), this, Axis.Y);
      Link rightFootLink = foot();
      rightAnklePitch.setLink(rightFootLink);
      rightAnkleRoll.addJoint(rightAnklePitch);

      GroundContactPoint right_toe_in = new GroundContactPoint("gc_right_toe_in",
                                           new Vector3D(m2Parameters.FOOT_FORWARD.value, m2Parameters.FOOT_WIDTH.value / 2.0, -m2Parameters.FOOT_HEIGHT.value),
                                           this);
      GroundContactPoint right_toe_out = new GroundContactPoint("gc_right_toe_out",
                                            new Vector3D(m2Parameters.FOOT_FORWARD.value, -m2Parameters.FOOT_WIDTH.value / 2.0,
                                               -m2Parameters.FOOT_HEIGHT.value), this);
      GroundContactPoint right_heel_in = new GroundContactPoint("gc_right_heel_in",
                                            new Vector3D(-m2Parameters.FOOT_BACK.value, m2Parameters.FOOT_WIDTH.value / 2.0, -m2Parameters.FOOT_HEIGHT.value),
                                            this);
      GroundContactPoint right_heel_out = new GroundContactPoint("gc_right_heel_out",
                                             new Vector3D(-m2Parameters.FOOT_BACK.value, -m2Parameters.FOOT_WIDTH.value / 2.0,
                                                -m2Parameters.FOOT_HEIGHT.value), this);

      rightAnklePitch.addGroundContactPoint(right_toe_in);
      rightAnklePitch.addGroundContactPoint(right_toe_out);
      rightAnklePitch.addGroundContactPoint(right_heel_in);
      rightAnklePitch.addGroundContactPoint(right_heel_out);

      // LEFT LEG.

      Joint leftHipUni = new UniversalJoint("left_hip_yaw", "left_hip_roll", new Vector3D(0.0, m2Parameters.HIP_OFFSET_Y.value, 0.0), this, Axis.Z, Axis.X);
      Link leftWaistLink = waist();
      leftHipUni.setLink(leftWaistLink);
      bodyJoint.addJoint(leftHipUni);

      Joint leftHipPitch = new PinJoint("left_hip_pitch", new Vector3D(0.0, 0.0, -m2Parameters.HIP_JOINT_OFF.value), this, Axis.Y);
      Link leftThighLink = leftThigh();
      leftHipPitch.setLink(leftThighLink);
      leftHipUni.addJoint(leftHipPitch);

      PinJoint leftKnee = new PinJoint("left_knee", new Vector3D(0.0, m2Parameters.HIP_TO_THIGH_OFF.value, -m2Parameters.THIGH_LENGTH.value), this, Axis.Y);
      Link leftShinLink = shin();
      leftKnee.setLink(leftShinLink);
      leftKnee.setLimitStops(0.0, Math.PI, 5000.0, 400.0);
      leftHipPitch.addJoint(leftKnee);

      Joint leftAnkleRoll = new PinJoint("left_ankle_roll", new Vector3D(0.0, 0.0, -m2Parameters.SHIN_LENGTH.value), this, Axis.X);
      Link leftRetinaculumLink = retinaculum();
      leftAnkleRoll.setLink(leftRetinaculumLink);
      leftKnee.addJoint(leftAnkleRoll);

      Joint leftAnklePitch = new PinJoint("left_ankle_pitch", new Vector3D(0.0, 0.0, -m2Parameters.ANKLE_JOINT_OFF.value), this, Axis.Y);
      Link leftFootLink = foot();
      leftAnklePitch.setLink(leftFootLink);
      leftAnkleRoll.addJoint(leftAnklePitch);

      GroundContactPoint left_toe_in = new GroundContactPoint("gc_left_toe_in",
                                          new Vector3D(m2Parameters.FOOT_FORWARD.value, -m2Parameters.FOOT_WIDTH.value / 2.0, -m2Parameters.FOOT_HEIGHT.value),
                                          this);
      GroundContactPoint left_toe_out = new GroundContactPoint("gc_left_toe_out",
                                           new Vector3D(m2Parameters.FOOT_FORWARD.value, m2Parameters.FOOT_WIDTH.value / 2.0, -m2Parameters.FOOT_HEIGHT.value),
                                           this);
      GroundContactPoint left_heel_in = new GroundContactPoint("gc_left_heel_in",
                                           new Vector3D(-m2Parameters.FOOT_BACK.value, -m2Parameters.FOOT_WIDTH.value / 2.0, -m2Parameters.FOOT_HEIGHT.value),
                                           this);
      GroundContactPoint left_heel_out = new GroundContactPoint("gc_left_heel_out",
                                            new Vector3D(-m2Parameters.FOOT_BACK.value, m2Parameters.FOOT_WIDTH.value / 2.0, -m2Parameters.FOOT_HEIGHT.value),
                                            this);

      leftAnklePitch.addGroundContactPoint(left_toe_in);
      leftAnklePitch.addGroundContactPoint(left_toe_out);
      leftAnklePitch.addGroundContactPoint(left_heel_in);
      leftAnklePitch.addGroundContactPoint(left_heel_out);

   }

   public void updateYawPitchRoll()
   {
      bodyJoint.getYawPitchRoll(q_yaw, q_pitch, q_roll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      bodyJoint.setYawPitchRoll(yaw, pitch, roll);
      updateYawPitchRoll();
   }

   private Link body()
   {
      Link ret = new Link("body");
      ret.setMass(m2Parameters.BODY_MASS.value);
      ret.setComOffset(m2Parameters.BODY_COM_X.value, m2Parameters.BODY_COM_Y.value, m2Parameters.BODY_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.BODY_Ixx.value, m2Parameters.BODY_Iyy.value, m2Parameters.BODY_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, m2Parameters.BODY_CYLINDER_HEIGHT.value);

      linkGraphics.addHemiEllipsoid(m2Parameters.BODY_R.value, m2Parameters.BODY_R.value, m2Parameters.BODY_ELLIPSE_HEIGHT.value, YoAppearance.Red());
      linkGraphics.translate(0.0, 0.0, -m2Parameters.BODY_CYLINDER_HEIGHT.value);
      linkGraphics.addCylinder(m2Parameters.BODY_CYLINDER_HEIGHT.value, m2Parameters.BODY_R.value);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link waist()
   {
      Link ret = new Link("waist");
      ret.setMass(m2Parameters.WAIST_MASS.value);
      ret.setComOffset(m2Parameters.WAIST_COM_X.value, m2Parameters.WAIST_COM_Y.value, m2Parameters.WAIST_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.WAIST_Ixx.value, m2Parameters.WAIST_Iyy.value, m2Parameters.WAIST_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(1.25f * m2Parameters.THIGH_R.value, YoAppearance.White());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link leftThigh()
   {
      Link ret = new Link("left_thigh");
      ret.setMass(m2Parameters.THIGH_MASS.value);
      ret.setComOffset(m2Parameters.THIGH_COM_X.value, m2Parameters.L_THIGH_COM_Y.value, m2Parameters.THIGH_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.THIGH_Ixx.value, m2Parameters.THIGH_Iyy.value, m2Parameters.THIGH_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, m2Parameters.HIP_TO_THIGH_OFF.value, -m2Parameters.THIGH_LENGTH.value);
      linkGraphics.addCylinder(m2Parameters.THIGH_LENGTH.value, m2Parameters.THIGH_R.value);
      linkGraphics.translate(0.0, -m2Parameters.HIP_TO_THIGH_OFF.value, m2Parameters.THIGH_LENGTH.value);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link rightThigh()
   {
      Link ret = new Link("right_thigh");
      ret.setMass(m2Parameters.THIGH_MASS.value);
      ret.setComOffset(m2Parameters.THIGH_COM_X.value, m2Parameters.R_THIGH_COM_Y.value, m2Parameters.THIGH_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.THIGH_Ixx.value, m2Parameters.THIGH_Iyy.value, m2Parameters.THIGH_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, -m2Parameters.HIP_TO_THIGH_OFF.value, -m2Parameters.THIGH_LENGTH.value);
      linkGraphics.addCylinder(m2Parameters.THIGH_LENGTH.value, m2Parameters.THIGH_R.value);
      linkGraphics.translate(0.0, m2Parameters.HIP_TO_THIGH_OFF.value, m2Parameters.THIGH_LENGTH.value);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link shin()
   {
      Link ret = new Link("shin");
      ret.setMass(m2Parameters.SHIN_MASS.value);
      ret.setComOffset(m2Parameters.SHIN_COM_X.value, m2Parameters.SHIN_COM_Y.value, m2Parameters.SHIN_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.SHIN_Ixx.value, m2Parameters.SHIN_Iyy.value, m2Parameters.SHIN_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(1.07f * m2Parameters.SHIN_R.value, YoAppearance.White());
      linkGraphics.translate(0.0, 0.0, -m2Parameters.SHIN_LENGTH.value);
      linkGraphics.addCylinder(m2Parameters.SHIN_LENGTH.value, m2Parameters.SHIN_R.value);
      linkGraphics.translate(0.0, 0.0, m2Parameters.SHIN_LENGTH.value);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link retinaculum()
   {
      Link ret = new Link("retinaculum");
      ret.setMass(m2Parameters.RETINACULUM_MASS.value);
      ret.setComOffset(m2Parameters.RETINACULUM_COM_X.value, m2Parameters.RETINACULUM_COM_Y.value, m2Parameters.RETINACULUM_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.RETINACULUM_Ixx.value, m2Parameters.RETINACULUM_Iyy.value, m2Parameters.RETINACULUM_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(m2Parameters.SHIN_R.value, YoAppearance.White());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link foot()
   {
      Link ret = new Link("foot");
      ret.setMass(m2Parameters.FOOT_MASS.value);
      ret.setComOffset(m2Parameters.FOOT_COM_X.value, m2Parameters.FOOT_COM_Y.value, m2Parameters.FOOT_COM_Z.value);
      ret.setMomentOfInertia(m2Parameters.FOOT_Ixx.value, m2Parameters.FOOT_Iyy.value, m2Parameters.FOOT_Izz.value);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(m2Parameters.FOOT_FORWARD.value - m2Parameters.FOOT_LENGTH.value / 2.0, 0.0, -m2Parameters.FOOT_HEIGHT.value);
      linkGraphics.addCube(m2Parameters.FOOT_LENGTH.value, m2Parameters.FOOT_WIDTH.value, m2Parameters.FOOT_HEIGHT.value, YoAppearance.Red());
      linkGraphics.translate(-m2Parameters.FOOT_FORWARD.value + m2Parameters.FOOT_LENGTH.value / 2.0, 0.0, m2Parameters.FOOT_HEIGHT.value);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
}
