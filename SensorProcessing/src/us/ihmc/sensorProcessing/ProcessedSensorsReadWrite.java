package us.ihmc.sensorProcessing;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.ProcessedIMUSensorsWriteOnlyInterface;
import us.ihmc.sensorProcessing.sensors.ProcessedIMUSensorsReadOnlyInterface;

public class ProcessedSensorsReadWrite implements ProcessedIMUSensorsReadOnlyInterface, ProcessedIMUSensorsWriteOnlyInterface
{
   protected final YoVariableRegistry registry = new YoVariableRegistry("ProcessedSensors");
   protected final YoDouble p_qs;
   protected final YoDouble p_qx;
   protected final YoDouble p_qy;
   protected final YoDouble p_qz;
   protected final YoDouble p_roll;
   protected final YoDouble p_pitch;
   protected final YoDouble p_yaw;
   protected final YoFrameVector pd_w;
   protected final YoFrameVector pdd_world;

   public ProcessedSensorsReadWrite(ReferenceFrame imuReferenceFrame, YoVariableRegistry yoVariableRegistry)
   {
      p_qs = new YoDouble("p_q", registry);
      p_qx = new YoDouble("p_qx", registry);
      p_qy = new YoDouble("p_qy", registry);
      p_qz = new YoDouble("p_qz", registry);

      p_pitch = new YoDouble("p_pitch", registry);
      p_roll = new YoDouble("p_roll", registry);
      p_yaw = new YoDouble("p_yaw", registry);

      pd_w = new YoFrameVector("pd_w", "", imuReferenceFrame, registry);
      pdd_world = new YoFrameVector("pdd_", "_world", ReferenceFrame.getWorldFrame(), registry);

      if (yoVariableRegistry != null)    // one of very few place where we can do this!
      {
         yoVariableRegistry.addChild(registry);
      }
   }

   public void setRotation(RotationMatrix rotationMatrix, int imuIndex)
   {
      Quaternion q = new Quaternion();
      q.set(rotationMatrix);

      // DON'T USE THIS: the method in Quat4d is flawed and doesn't work for some rotation matrices!
//      q.set(rotationMatrix);

      p_qs.set(q.getS());
      p_qx.set(q.getX());
      p_qy.set(q.getY());
      p_qz.set(q.getZ());

      p_yaw.set(rotationMatrix.getYaw());
      p_pitch.set(rotationMatrix.getPitch());
      p_roll.set(rotationMatrix.getRoll());
   }

   public void setAcceleration(FrameVector accelerationInWorld, int imuIndex)
   {
      pdd_world.set(accelerationInWorld);
   }

   public void setAngularVelocityInBody(Vector3D angularVelocityInBody, int imuIndex)
   {
      pd_w.set(angularVelocityInBody);
   }

   public void setAngularAccelerationInBody(Vector3D angularAccelerationInBody, int imuIndex)
   {
      throw new RuntimeException("Not supported");
   }

   public Quaternion getQuaternion(int imuIndex)
   {
      return new Quaternion(p_qx.getDoubleValue(), p_qy.getDoubleValue(), p_qz.getDoubleValue(),
                        p_qs.getDoubleValue());
   }

   public FrameVector getAcceleration(int imuIndex)
   {
      return pdd_world.getFrameVectorCopy();
   }

   public FrameVector getAngularVelocity(int imuIndex)
   {
      return pd_w.getFrameVectorCopy();
   }

   public double getYaw(int imuIndex)
   {
      return p_yaw.getDoubleValue();
   }

   public double getPitch(int imuIndex)
   {
      return p_pitch.getDoubleValue();
   }

   public double getRoll(int imuIndex)
   {
      return p_roll.getDoubleValue();
   }

   public FrameVector getAngularAcceleration(int imuIndex)
   {
      throw new RuntimeException("Not implemented");
   }
}
