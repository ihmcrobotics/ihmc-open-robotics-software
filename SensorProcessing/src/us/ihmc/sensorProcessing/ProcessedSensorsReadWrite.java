package us.ihmc.sensorProcessing;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.ProcessedIMUSensorsWriteOnlyInterface;
import us.ihmc.sensorProcessing.sensors.ProcessedIMUSensorsReadOnlyInterface;

public class ProcessedSensorsReadWrite implements ProcessedIMUSensorsReadOnlyInterface, ProcessedIMUSensorsWriteOnlyInterface
{
   protected final YoVariableRegistry registry = new YoVariableRegistry("ProcessedSensors");
   protected final DoubleYoVariable p_qs;
   protected final DoubleYoVariable p_qx;
   protected final DoubleYoVariable p_qy;
   protected final DoubleYoVariable p_qz;
   protected final DoubleYoVariable p_roll;
   protected final DoubleYoVariable p_pitch;
   protected final DoubleYoVariable p_yaw;
   protected final YoFrameVector pd_w;
   protected final YoFrameVector pdd_world;

   public ProcessedSensorsReadWrite(ReferenceFrame imuReferenceFrame, YoVariableRegistry yoVariableRegistry)
   {
      p_qs = new DoubleYoVariable("p_q", registry);
      p_qx = new DoubleYoVariable("p_qx", registry);
      p_qy = new DoubleYoVariable("p_qy", registry);
      p_qz = new DoubleYoVariable("p_qz", registry);

      p_pitch = new DoubleYoVariable("p_pitch", registry);
      p_roll = new DoubleYoVariable("p_roll", registry);
      p_yaw = new DoubleYoVariable("p_yaw", registry);

      pd_w = new YoFrameVector("pd_w", "", imuReferenceFrame, registry);
      pdd_world = new YoFrameVector("pdd_", "_world", ReferenceFrame.getWorldFrame(), registry);

      if (yoVariableRegistry != null)    // one of very few place where we can do this!
      {
         yoVariableRegistry.addChild(registry);
      }
   }

   public void setRotation(Matrix3d rotationMatrix, int imuIndex)
   {
      Quat4d q = new Quat4d();
      RotationTools.convertMatrixToQuaternion(rotationMatrix, q);
      RotationTools.checkQuaternionNormalized(q);

      // DON'T USE THIS: the method in Quat4d is flawed and doesn't work for some rotation matrices!
//      q.set(rotationMatrix);

      p_qs.set(q.getW());
      p_qx.set(q.getX());
      p_qy.set(q.getY());
      p_qz.set(q.getZ());

      p_yaw.set(RotationTools.computeYaw(rotationMatrix));
      p_pitch.set(RotationTools.computePitch(rotationMatrix));
      p_roll.set(RotationTools.computeRoll(rotationMatrix));
   }

   public void setAcceleration(FrameVector accelerationInWorld, int imuIndex)
   {
      pdd_world.set(accelerationInWorld);
   }

   public void setAngularVelocityInBody(Vector3d angularVelocityInBody, int imuIndex)
   {
      pd_w.set(angularVelocityInBody);
   }

   public void setAngularAccelerationInBody(Vector3d angularAccelerationInBody, int imuIndex)
   {
      throw new RuntimeException("Not supported");
   }

   public Quat4d getQuaternion(int imuIndex)
   {
      return new Quat4d(p_qx.getDoubleValue(), p_qy.getDoubleValue(), p_qz.getDoubleValue(),
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
