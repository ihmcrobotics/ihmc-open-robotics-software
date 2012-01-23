package com.yobotics.simulationconstructionset.processedSensors;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public interface ProcessedIMUSensorsReadOnlyInterface
{
   public abstract Quat4d getQuaternion(int imuIndex);
   
   public abstract DoubleYoVariable[] getYawPitchRoll(int imuIndex);

   public abstract FrameVector getBodyAcceleration(int imuIndex);
   
   public abstract Vector3d getBodyAngularVelocity(int imuIndex);
}