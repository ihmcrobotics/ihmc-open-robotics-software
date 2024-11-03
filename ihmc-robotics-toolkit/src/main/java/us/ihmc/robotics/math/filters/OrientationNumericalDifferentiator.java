package us.ihmc.robotics.math.filters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.RotationTools;

public class OrientationNumericalDifferentiator
{
   public static FrameVector3D differentiate(double dt, ReferenceFrame frame, FrameQuaternion before, FrameQuaternion current, FrameQuaternion after)
   {
      ReferenceFrame beforeInitialFrame = before.getReferenceFrame();
      ReferenceFrame currentInitialFrame = current.getReferenceFrame();
      ReferenceFrame afterInitialFrame = after.getReferenceFrame();

      before.changeFrame(frame);
      current.changeFrame(frame);
      after.changeFrame(frame);

      // R^o_(bt-1) = before
      RotationMatrix oRbtm1 = new RotationMatrix(before);

      // R^(bt-1)_o = R^o_(bt-1)^-1
      RotationMatrix btm1Ro = new RotationMatrix();
      btm1Ro.setAndInvert(oRbtm1);

      // R^o_(bt+1) = after
      RotationMatrix oRbt1 = new RotationMatrix(after);

      // R^(bt-1)_(bt+1) = R^(bt-1)_o * R^o_(bt+1)
      RotationMatrix btRbt1 = new RotationMatrix();
      btRbt1.set(btm1Ro);
      btRbt1.multiply(oRbt1);

      AxisAngle w = new AxisAngle();
      AxisAngleConversion.convertMatrixToAxisAngle(btRbt1, w);

      // v_bt = [w/||w||, ||w||/(2*dt)]
      Vector3D vbt = new Vector3D(w.getX(), w.getY(), w.getZ());
      vbt.normalize();
      vbt.scale(w.getAngle() / (2 * dt));

      // R^o_bt = current
      RotationMatrix oRbt = new RotationMatrix(current);

      // v_wt = R^o_bt * v_bt
      Vector3D vwt = mul3(oRbt, vbt);

      before.changeFrame(beforeInitialFrame);
      current.changeFrame(currentInitialFrame);
      after.changeFrame(afterInitialFrame);

      return new FrameVector3D(frame, vwt);
   }

   public static List<FrameVector3D> differentiate(double dt, ReferenceFrame frame, List<FrameQuaternion> orientations)
   {
      List<FrameVector3D> vs = new ArrayList<>(orientations.size());
      for (int i = 1; i < orientations.size() - 1; i++)
      {
         FrameVector3D v = differentiate(dt, frame, orientations.get(i - 1), orientations.get(i), orientations.get(i + 1));
         vs.add(v);
      }

      // Add endpoints by padding out elements 1 and n-1
      vs.add(0, new FrameVector3D(vs.get(0)));
      vs.add(new FrameVector3D(vs.get(vs.size() - 1)));

      return vs;
   }

   /**
    * @see <a href="https://ocw.mit.edu/courses/mechanical-engineering/2-017j-design-of-electromechanical-robotic-systems-fall-2009/course-text/MIT2_017JF09_ch09.pdf}">KINEMATICS
    * OF MOVING FRAMES</a>, pg. 71.
    */
   public static Vector3D differentiate2(double dt, ReferenceFrame frame, FrameQuaternion before, FrameQuaternion current, FrameQuaternion after)
   {
      ReferenceFrame beforeInitialFrame = before.getReferenceFrame();
      ReferenceFrame currentInitialFrame = current.getReferenceFrame();
      ReferenceFrame afterInitialFrame = after.getReferenceFrame();

      before.changeFrame(frame);
      current.changeFrame(frame);
      after.changeFrame(frame);

      double yaw = current.getYaw();
      double pitch = current.getPitch();
      double roll = current.getRoll();

      double dyaw = (after.getYaw() - before.getYaw()) / (2.0 * dt);
      double dpitch = (after.getPitch() - before.getPitch()) / (2.0 * dt);
      double droll = (after.getRoll() - before.getRoll()) / (2.0 * dt);

      Matrix3D R = new Matrix3D();
      R.set(1, 0, -Math.sin(pitch),
            0, Math.cos(roll), Math.sin(roll) * Math.cos(pitch),
            0, -Math.sin(roll), Math.cos(roll) * Math.cos(pitch));

      Vector3D vE = new Vector3D(droll, dpitch, dyaw);
      Vector3D vbt = mul3(R, vE);

      RotationMatrix oRbt = new RotationMatrix(current);

      Vector3D vwt = mul3(oRbt, vbt);

      before.changeFrame(beforeInitialFrame);
      current.changeFrame(currentInitialFrame);
      after.changeFrame(afterInitialFrame);

      return vwt;
   }

   private static Vector3D mul3(Matrix3DReadOnly M, Vector3DReadOnly v)
   {
      Matrix3D V = new Matrix3D();
      V.set(v.getX(), v.getX(), v.getX(),
            v.getY(), v.getY(), v.getY(),
            v.getZ(), v.getZ(), v.getZ());

      Matrix3D MV = new Matrix3D();
      MV.set(M);
      MV.multiply(V);

      return new Vector3D(MV.getM00(), MV.getM11(), MV.getM22());
   }

   public static void main(String[] args)
   {
      Vector3D initialVelocity = new Vector3D(1.00, 1.00, 1.00);
      double time = 0.01;
      RotationMatrix initialRotation = new RotationMatrix();
      initialRotation.setYawPitchRoll(1, 1, 1);

      // compute m1 in o
      RotationMatrix m1 = new RotationMatrix();
      m1.set(initialRotation);

      // compute m2 in m1
      RotationMatrix m1inv = new RotationMatrix();
      m1inv.setAndInvert(m1);

      RotationMatrix m21 = new RotationMatrix();
      Vector3D velIn1 = mul3(m1inv, initialVelocity);
      RotationTools.integrateAngularVelocity(velIn1, time, m21);

      // compute m2 in o
      RotationMatrix m2 = new RotationMatrix();
      m2.set(m1);
      m2.multiply(m21);

      RotationMatrix m2inv = new RotationMatrix();
      m2inv.setAndInvert(m2);

      // compute m3 in m2
      RotationMatrix m32 = new RotationMatrix();
      Vector3D velIn2 = mul3(m2inv, initialVelocity);
      RotationTools.integrateAngularVelocity(velIn2, time, m32);

      RotationMatrix m3 = new RotationMatrix();
      m3.set(m2);
      m3.multiply(m32);

      FrameQuaternion f1 = new FrameQuaternion(ReferenceFrame.getWorldFrame(), m1);
      FrameQuaternion f2 = new FrameQuaternion(ReferenceFrame.getWorldFrame(), m2);
      FrameQuaternion f3 = new FrameQuaternion(ReferenceFrame.getWorldFrame(), m3);

      System.out.println(f1);
      System.out.println(f2);
      System.out.println(f3);

      System.out.println(initialVelocity);
      System.out.println(differentiate(0.01, ReferenceFrame.getWorldFrame(), f1, f2, f3));
      System.out.println(differentiate2(0.01, ReferenceFrame.getWorldFrame(), f1, f2, f3));
   }
}
