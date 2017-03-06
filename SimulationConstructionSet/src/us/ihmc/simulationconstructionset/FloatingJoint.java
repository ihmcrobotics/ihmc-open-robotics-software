package us.ihmc.simulationconstructionset;

import net.jafama.FastMath;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.FloatJointPhysics;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */
public class FloatingJoint extends Joint implements FloatingSCSJoint
{
   private static final long serialVersionUID = 6863566500545068060L;

   private Quaternion tempOrientation1 = new Quaternion();
   private Vector3D tempPosition1 = new Vector3D();

   public DoubleYoVariable q_x, q_y, q_z;    // in world-fixed frame
   public DoubleYoVariable qd_x;
   public DoubleYoVariable qd_y;
   public DoubleYoVariable qd_z;    // in world-fixed frame
   public DoubleYoVariable q_qs;
   public DoubleYoVariable q_qx;
   public DoubleYoVariable q_qy;
   public DoubleYoVariable q_qz;    // Unit quaternion (Euler parameters). q_qs is the 'scalar part', q_q{x,y,z} form the vector part
   public DoubleYoVariable qd_wx;
   public DoubleYoVariable qd_wy;
   public DoubleYoVariable qd_wz;    // angular velocity, expressed in body-fixed frame.
   public DoubleYoVariable qdd_x, qdd_y, qdd_z;    // in world-fixed frame
   public DoubleYoVariable qdd_wx, qdd_wy, qdd_wz;    // angular acceleration, expressed in body-fixed frame.

   private final boolean createYawPitchRollYoVariable;
   public DoubleYoVariable q_yaw, q_pitch, q_roll;    // in world-fixed frame.
   
   public FloatingJoint(String jname, Vector3D offset, Robot rob)
   {
      this(jname, null, offset, rob, false);
   }

   public FloatingJoint(String jname, Vector3D offset, Robot rob, boolean createYawPitchRollYoVariable)
   {
      this(jname, null, offset, rob, createYawPitchRollYoVariable);
   }

   public FloatingJoint(String jname, String varName, Vector3D offset, Robot rob)
   {
      this(jname, varName, offset, rob, false);
   }
   
   public FloatingJoint(String jname, String varName, Vector3D offset, Robot rob, boolean createYawPitchRollYoVariable)
   {
      super(jname, offset, rob, 6);

      physics = new FloatJointPhysics(this);

      YoVariableRegistry registry = rob.getRobotsYoVariableRegistry();

      this.createYawPitchRollYoVariable = createYawPitchRollYoVariable;
      
      if (varName == null)
      {
         varName = "";
      }
      else if (!varName.isEmpty())
      {
         varName += "_";
      }
      
      q_x = new DoubleYoVariable("q_" + varName + "x", "FloatingJoint x position", registry);
      q_y = new DoubleYoVariable("q_" + varName + "y", "FloatingJoint y position", registry);
      q_z = new DoubleYoVariable("q_" + varName + "z", "FloatingJoint z position", registry);
      qd_x = new DoubleYoVariable("qd_" + varName + "x", "FloatingJoint x velocity", registry);
      qd_y = new DoubleYoVariable("qd_" + varName + "y", "FloatingJoint y velocity", registry);
      qd_z = new DoubleYoVariable("qd_" + varName + "z", "FloatingJoint z velocity", registry);
      qdd_x = new DoubleYoVariable("qdd_" + varName + "x", "FloatingJoint x acceleration", registry);
      qdd_y = new DoubleYoVariable("qdd_" + varName + "y", "FloatingJoint yx acceleration", registry);
      qdd_z = new DoubleYoVariable("qdd_" + varName + "z", "FloatingJoint z acceleration", registry);
      q_qs = new DoubleYoVariable("q_" + varName + "qs", "FloatingJoint orientation quaternion qs", registry);
      q_qs.set(1.0);
      q_qx = new DoubleYoVariable("q_" + varName + "qx", "FloatingJoint orientation quaternion qx", registry);
      q_qy = new DoubleYoVariable("q_" + varName + "qy", "FloatingJoint orientation quaternion qy", registry);
      q_qz = new DoubleYoVariable("q_" + varName + "qz", "FloatingJoint orientation quaternion qz", registry);
      qd_wx = new DoubleYoVariable("qd_" + varName + "wx", "FloatingJoint rotational velocity about x", registry);
      qd_wy = new DoubleYoVariable("qd_" + varName + "wy", "FloatingJoint rotational velocity about y", registry);
      qd_wz = new DoubleYoVariable("qd_" + varName + "wz", "FloatingJoint rotational velocity about z", registry);
      qdd_wx = new DoubleYoVariable("qdd_" + varName + "wx", "FloatingJoint rotational acceleration about x", registry);
      qdd_wy = new DoubleYoVariable("qdd_" + varName + "wy", "FloatingJoint rotational acceleration about y", registry);
      qdd_wz = new DoubleYoVariable("qdd_" + varName + "wz", "FloatingJoint rotational acceleration about z", registry);

      if(createYawPitchRollYoVariable)
      {
         q_yaw = new DoubleYoVariable("q_" + varName + "yaw", "FloatingJoint yaw orientation", registry);
         q_pitch = new DoubleYoVariable("q_" + varName + "pitch", "FloatingJoint pitch orientation", registry);
         q_roll = new DoubleYoVariable("q_" + varName + "roll", "FloatingJoint roll orientation", registry);
      }
      else
      {
         q_yaw = null;
         q_pitch = null;
         q_roll = null;
      }
      
      this.setFloatingTransform3D(this.jointTransform3D);
      physics.u_i = null;
   }

   public void setPositionAndVelocity(double x, double y, double z, double dx, double dy, double dz)
   {
      q_x.set(x);
      q_y.set(y);
      q_z.set(z);
      qd_x.set(dx);
      qd_y.set(dy);
      qd_z.set(dz);
   }

   public void setPositionAndVelocity(Tuple3DBasics position, Tuple3DBasics velocity)
   {
      q_x.set(position.getX());
      q_y.set(position.getY());
      q_z.set(position.getZ());
      qd_x.set(velocity.getX());
      qd_y.set(velocity.getY());
      qd_z.set(velocity.getZ());
   }

   public void setPosition(Tuple3DBasics position)
   {
      q_x.set(position.getX());
      q_y.set(position.getY());
      q_z.set(position.getZ());
   }
   
   public void setPosition(double x, double y, double z)
   {
      q_x.set(x);
      q_y.set(y);
      q_z.set(z);
   }

   @Override
   public void setVelocity(Tuple3DBasics velocity)
   {
      qd_x.set(velocity.getX());
      qd_y.set(velocity.getY());
      qd_z.set(velocity.getZ());
   }
   
   public void setVelocity(double xd, double yd, double zd)
   {
      qd_x.set(xd);
      qd_y.set(yd);
      qd_z.set(zd);
   }

   public void setAcceleration(Tuple3DBasics acceleration)
   {
      qdd_x.set(acceleration.getX());
      qdd_y.set(acceleration.getY());
      qdd_z.set(acceleration.getZ());
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      Quaternion q = new Quaternion();
      q.setYawPitchRoll(yaw, pitch, roll);
      q.checkIfUnitary();
      q_qs.set(q.getS());
      q_qx.set(q.getX());
      q_qy.set(q.getY());
      q_qz.set(q.getZ());
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll, double wz, double wy, double wx)
   {
      setYawPitchRoll(yaw, pitch, roll);
      qd_wz.set(wz);
      qd_wy.set(wy);
      qd_wx.set(wx);
   }

   public void setRotation(RotationMatrix rotation)
   {
      double r11, r12, r13, r21, r22, r23, r31, r32, r33;

      r11 = rotation.getM00();
      r12 = rotation.getM01();
      r13 = rotation.getM02();
      r21 = rotation.getM10();
      r22 = rotation.getM11();
      r23 = rotation.getM12();
      r31 = rotation.getM20();
      r32 = rotation.getM21();
      r33 = rotation.getM22();
      q_qs.set(Math.sqrt((1.0 + r11 + r22 + r33) / 4.0));
      q_qx.set((r32 - r23) / (4.0 * q_qs.getDoubleValue()));
      q_qy.set((r13 - r31) / (4.0 * q_qs.getDoubleValue()));
      q_qz.set((r21 - r12) / (4.0 * q_qs.getDoubleValue()));
   }

   public void setQuaternion(Quaternion q)
   {
      q_qs.set(q.getS());
      q_qx.set(q.getX());
      q_qy.set(q.getY());
      q_qz.set(q.getZ());
   }

   @Override
   public void setRotationAndTranslation(RigidBodyTransform transform)
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      transform.getRotation(rotationMatrix);
      setRotation(rotationMatrix);

      Vector3D translation = new Vector3D();
      transform.getTranslation(translation);
      setPosition(translation);
   }

   @Override
   public void setAngularVelocityInBody(Vector3D angularVelocityInBody)
   {
      qd_wx.set(angularVelocityInBody.getX());
      qd_wy.set(angularVelocityInBody.getY());
      qd_wz.set(angularVelocityInBody.getZ());
   }

   public void setAngularAccelerationInBody(Vector3D angularAccelerationInBody)
   {
      qdd_wx.set(angularAccelerationInBody.getX());
      qdd_wy.set(angularAccelerationInBody.getY());
      qdd_wz.set(angularAccelerationInBody.getZ());
   }

   public void getPosition(DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable z)
   {
      x.set(q_x.getDoubleValue());
      y.set(q_y.getDoubleValue());
      z.set(q_z.getDoubleValue());
   }

   public void getVelocity(DoubleYoVariable xDot, DoubleYoVariable yDot, DoubleYoVariable zDot)
   {
      xDot.set(qd_x.getDoubleValue());
      yDot.set(qd_y.getDoubleValue());
      zDot.set(qd_z.getDoubleValue());
   }
   
   @Override
   public void getVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), qd_x.getDoubleValue(), qd_y.getDoubleValue(), qd_z.getDoubleValue());
   }
   

   @Override
   public void getAngularVelocity(FrameVector angularVelocityToPack, ReferenceFrame bodyFrame)
   {
      angularVelocityToPack.setIncludingFrame(bodyFrame, qd_wx.getDoubleValue(), qd_wy.getDoubleValue(), qd_wz.getDoubleValue());
   }

   public void getPositionAndVelocity(DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable z, DoubleYoVariable xDot, DoubleYoVariable yDot, DoubleYoVariable zDot)
   {
      getPosition(x, y, z);
      getVelocity(xDot, yDot, zDot);
   }

   public void getPosition(Tuple3DBasics position)
   {
      position.set(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue());
   }
   
   public void getVelocity(Tuple3DBasics velocity)
   {
      velocity.set(qd_x.getDoubleValue(), qd_y.getDoubleValue(), qd_z.getDoubleValue());
   }

   public void getPositionAndVelocity(Tuple3DBasics position, Tuple3DBasics velocity)
   {
      getPosition(position);
      getVelocity(velocity);
   }

   public DoubleYoVariable getQx()
   {
      return q_x;
   }

   public DoubleYoVariable getQy()
   {
      return q_y;
   }

   public DoubleYoVariable getQz()
   {
      return q_z;
   }

   public DoubleYoVariable getQdx()
   {
      return qd_x;
   }

   public DoubleYoVariable getQdy()
   {
      return qd_y;
   }

   public DoubleYoVariable getQdz()
   {
      return qd_z;
   }

   public DoubleYoVariable getQddx()
   {
      return qdd_x;
   }

   public DoubleYoVariable getQddy()
   {
      return qdd_y;
   }

   public DoubleYoVariable getQddz()
   {
      return qdd_z;
   }

   public DoubleYoVariable getQuaternionQs()
   {
      return q_qs;
   }

   public DoubleYoVariable getQuaternionQx()
   {
      return q_qx;
   }

   public DoubleYoVariable getQuaternionQy()
   {
      return q_qy;
   }

   public DoubleYoVariable getQuaternionQz()
   {
      return q_qz;
   }

   public Quaternion getQuaternion()
   {
      return new Quaternion(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue());
   }
   
   public void getQuaternion(Quaternion quaternionToPack)
   {
      quaternionToPack.set(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue());
   }

   public DoubleYoVariable getAngularVelocityX()
   {
      return qd_wx;
   }

   public DoubleYoVariable getAngularVelocityY()
   {
      return qd_wy;
   }

   public DoubleYoVariable getAngularVelocityZ()
   {
      return qd_wz;
   }

   public Vector3D getAngularVelocityInBody()
   {
      return new Vector3D(qd_wx.getDoubleValue(), qd_wy.getDoubleValue(), qd_wz.getDoubleValue());
   }
   
   public void getAngularVelocityInBody(Vector3D vectorToPack)
   {
      vectorToPack.set(qd_wx.getDoubleValue(), qd_wy.getDoubleValue(), qd_wz.getDoubleValue());
   }

   public DoubleYoVariable getAngularAccelerationX()
   {
      return qdd_wx;
   }

   public DoubleYoVariable getAngularAccelerationY()
   {
      return qdd_wy;
   }

   public DoubleYoVariable getAngularAccelerationZ()
   {
      return qdd_wz;
   }

   public Vector3D getAngularAccelerationInBody()
   {
      return new Vector3D(qdd_wx.getDoubleValue(), qdd_wy.getDoubleValue(), qdd_wz.getDoubleValue());
   }
   
   public void getAngularAccelerationInBody(Vector3D angularAccelerationInBodyToPack)
   {
      angularAccelerationInBodyToPack.set(qdd_wx.getDoubleValue(), qdd_wy.getDoubleValue(), qdd_wz.getDoubleValue());
   }

   public void getAngularAcceleration(FrameVector angularAccelerationToPack, ReferenceFrame bodyFrame)
   {
      angularAccelerationToPack.setIncludingFrame(bodyFrame, qdd_wx.getDoubleValue(), qdd_wy.getDoubleValue(), qdd_wz.getDoubleValue());      
   }
   
   public void getLinearAccelerationInWorld(Vector3D accelerationInWorldToPack)
   {
      accelerationInWorldToPack.set(qdd_x.getDoubleValue(), qdd_y.getDoubleValue(), qdd_z.getDoubleValue()); 
   }
   
   public void getLinearAcceleration(FrameVector linearAccelerationToPack)
   {
      linearAccelerationToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), qdd_x.getDoubleValue(), qdd_y.getDoubleValue(), qdd_z.getDoubleValue()); 
   }

   public void getYawPitchRoll(DoubleYoVariable yaw, DoubleYoVariable pitch, DoubleYoVariable roll)
   {

      double pitchArgument = -2.0 * q_qx.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qs.getDoubleValue() * q_qy.getDoubleValue();

      pitch.set(FastMath.asin(pitchArgument));

      if (Math.abs(pitch.getDoubleValue()) < 0.49 * Math.PI)
      {
         yaw.set(FastMath.atan2(2.0 * q_qx.getDoubleValue() * q_qy.getDoubleValue() + 2.0 * q_qz.getDoubleValue() * q_qs.getDoubleValue(),
                            1.0 - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue() - 2.0 * q_qz.getDoubleValue() * q_qz.getDoubleValue()));    // Math.asin(q_qs.val * q_qz.val * 2.0);
         roll.set(FastMath.atan2(2.0 * q_qy.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qx.getDoubleValue() * q_qs.getDoubleValue(),
                             1.0 - 2.0 * q_qx.getDoubleValue() * q_qx.getDoubleValue() - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue()));    // Math.asin(q_qs.val * q_qx.val * 2.0);
      }
      else
      {
         yaw.set(2.0 * FastMath.atan2(q_qz.getDoubleValue(), q_qs.getDoubleValue()));
         roll.set(0.0);
      }
   }
   
   public double[] getYawPitchRoll()
   {
      double[] yawPitchRollToReturn = new double[3];
      
      double pitchArgument = -2.0 * q_qx.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qs.getDoubleValue() * q_qy.getDoubleValue();

      double pitch = 0.0, roll = 0.0, yaw = 0.0;
      
      pitch = FastMath.asin(pitchArgument);

      if (Math.abs(pitch) < 0.49 * Math.PI)
      {
         yaw = FastMath.atan2(2.0 * q_qx.getDoubleValue() * q_qy.getDoubleValue() + 2.0 * q_qz.getDoubleValue() * q_qs.getDoubleValue(),
                            1.0 - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue() - 2.0 * q_qz.getDoubleValue() * q_qz.getDoubleValue());    // Math.asin(q_qs.val * q_qz.val * 2.0);
         roll = FastMath.atan2(2.0 * q_qy.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qx.getDoubleValue() * q_qs.getDoubleValue(),
                             1.0 - 2.0 * q_qx.getDoubleValue() * q_qx.getDoubleValue() - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue());    // Math.asin(q_qs.val * q_qx.val * 2.0);
      }
      else
      {
         yaw = 2.0 * FastMath.atan2(q_qz.getDoubleValue(), q_qs.getDoubleValue());
         roll = 0.0;
      }
      
      yawPitchRollToReturn[0] = yaw;
      yawPitchRollToReturn[1] = pitch;
      yawPitchRollToReturn[2] = roll;
      
      return yawPitchRollToReturn;
   }

   @Override
   public void update()
   {
      this.setFloatingTransform3D(this.jointTransform3D);
      if(createYawPitchRollYoVariable)
      {
         getYawPitchRoll(q_yaw, q_pitch, q_roll);
      }
   }

// private Matrix3d tempRotationMatrix = new Matrix3d();
   protected void setFloatingTransform3D(RigidBodyTransform t1)
   {
      // position.set(q_x.val + offset.x, q_y.val + offset.y, q_z.val + offset.z);
      tempPosition1.set(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue());
      tempOrientation1.set(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue());
      t1.set(tempOrientation1, tempPosition1);

//    // An alternate way is to hardcode from http://www.genesis3d.com/~kdtop/Quaternions-UsingToRepresentRotation.htm (except do the transpose):
//    double w2 = q_qs.val * q_qs.val;
//    double x2 = q_qx.val * q_qx.val;
//    double y2 = q_qy.val * q_qy.val;
//    double z2 = q_qz.val * q_qz.val;
//
//    double wx = q_qs.val * q_qx.val;
//    double wy = q_qs.val * q_qy.val;
//    double wz = q_qs.val * q_qz.val;
//
//    double xy = q_qx.val * q_qy.val;
//    double xz = q_qx.val * q_qz.val;
//    double yz = q_qy.val * q_qz.val;
//
//
////    Matrix3d rotationMatrix = new Matrix3d
//
//        tempRotationMatrix = new Matrix3d(
//        w2+x2-y2-z2, 2.0*(xy - wz), 2.0*(xz + wy),
//        2.0*(xy+wz), w2-x2+y2-z2, 2.0*(yz-wx),
//        2.0*(xz-wy), 2.0*(yz+wx), w2-x2-y2+z2
//        );
//
//    t1.set(tempRotationMatrix, tempPosition1, 1.0);
   }

}
