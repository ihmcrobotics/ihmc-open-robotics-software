package us.ihmc.simulationconstructionset;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class IMUMount
{
   private final String name;

   private RigidBodyTransform transformFromMountToJoint;

   private Joint parentJoint;

   private Robot robot;

   private final YoFrameVector angularVelocityInBody;
   private final YoFrameVector angularAccelerationInBody;

   private final YoFrameVector linearVelocityInBody;
   private final YoFrameVector linearVelocityInWorld;

   private final YoFrameVector linearAccelerationInBody;
   private final YoFrameVector linearAccelerationInWorld;

   private final YoFrameQuaternion orientation;
   
   private double accelerationGaussianNoiseMean = 0.0, accelerationGaussianNoiseStdDev = 0.0;
   private double accelerationGaussianBiasMean = 0.0, accelerationGaussianBiasStdDev = 0.0;
   
   private double angularVelocityGaussianNoiseMean = 0.0, angularVelocityGaussianNoiseStdDev = 0.0;
   private double angularVelocityGaussianBiasMean = 0.0, angularVelocityGaussianBiasStdDev = 0.0;

   public IMUMount(String name, RigidBodyTransform offset, Robot robot)
   {
      this.name = name;
      this.robot = robot;

      transformFromMountToJoint = new RigidBodyTransform(offset);

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();

      orientation = new YoFrameQuaternion(name + "Orientation", null, registry);

      linearVelocityInBody = new YoFrameVector(name + "LinearVelocity", null, registry);
      linearVelocityInWorld = new YoFrameVector(name + "LinearVelocityWorld", null, registry);

      angularVelocityInBody = new YoFrameVector(name + "AngularVelocity", null, registry);
      angularAccelerationInBody = new YoFrameVector(name + "AngularAcceleration", null, registry);

      linearAccelerationInBody = new YoFrameVector(name + "LinearAcceleration", null, registry);
      linearAccelerationInWorld = new YoFrameVector(name + "LinearAccelerationWorld", null, registry);
   }

   public String getName()
   {
      return name;
   }

   protected void setParentJoint(Joint parent)
   {
      this.parentJoint = parent;
   }

   public Joint getParentJoint()
   {
      return parentJoint;
   }

   private final Vector3d tempLinearVelocity = new Vector3d();
   private final Vector3d tempAngularVelocityInBody = new Vector3d();

   private final Matrix3d tempRotationToWorld = new Matrix3d();
   private final Vector3d tempIMUOffset = new Vector3d();
   private final Matrix3d tempIMURotation = new Matrix3d();

   protected void updateIMUMountPositionAndVelocity()
   {
      // Offsets of IMUMount:
      transformFromMountToJoint.get(tempIMUOffset);
      transformFromMountToJoint.get(tempIMURotation);

      // Orientation:
      parentJoint.getRotationToWorld(tempRotationToWorld);
      tempRotationToWorld.mul(tempIMURotation);
      orientation.set(tempRotationToWorld);

      tempIMURotation.transpose();

      // TODO: These are the values stored from whatever the last stage of the integrator did.
      // They do not get averaged with the RungeKutta (or other integrator) averager.

      // Linear Velocity
      parentJoint.physics.getLinearVelocityInBody(tempLinearVelocity, tempIMUOffset);
      tempIMURotation.transform(tempLinearVelocity);
      linearVelocityInBody.set(tempLinearVelocity);
      parentJoint.physics.getLinearVelocityInWorld(tempLinearVelocity, tempIMUOffset);
      linearVelocityInWorld.set(tempLinearVelocity);

      // Angular Velocity
      parentJoint.physics.getAngularVelocityInBody(tempAngularVelocityInBody);
      tempIMURotation.transform(tempAngularVelocityInBody);
      angularVelocityInBody.set(tempAngularVelocityInBody);
   }

   private final Vector3d tempGravity = new Vector3d();
   private final Vector3d tempLinearAcceleration = new Vector3d();
   private final Vector3d tempAngularAccelerationInBody = new Vector3d();
   
   protected void updateIMUMountAcceleration()
   {
      // We redo some of the things that are already done in updateIMUMountPositionAndVelocity, 
      // but it is safer that way since updateIMUMountAcceleration might be called by itself sometimes.
      
      // Offsets of IMUMount:
      transformFromMountToJoint.get(tempIMUOffset);
      transformFromMountToJoint.get(tempIMURotation);

      // Orientation:
      parentJoint.getRotationToWorld(tempRotationToWorld);
      tempRotationToWorld.mul(tempIMURotation);
      tempIMURotation.transpose();
      
      // Linear Acceleration
      parentJoint.physics.getLinearAccelerationInBody(tempLinearAcceleration, tempIMUOffset);
      tempIMURotation.transform(tempLinearAcceleration);

      robot.getGravity(tempGravity);
      tempGravity.scale(-1.0);

      tempRotationToWorld.transpose();
      tempRotationToWorld.transform(tempGravity);
      tempLinearAcceleration.add(tempGravity);
      linearAccelerationInBody.set(tempLinearAcceleration);

      parentJoint.physics.getLinearAccelerationInWorld(tempLinearAcceleration, tempIMUOffset);
      robot.getGravity(tempGravity);
      tempGravity.scale(-1.0);

      tempLinearAcceleration.add(tempGravity);
      linearAccelerationInWorld.set(tempLinearAcceleration);

      // Angular Acceleration
      parentJoint.physics.getAngularAccelerationsInBodyFrame(tempAngularAccelerationInBody);
      tempIMURotation.transform(tempAngularAccelerationInBody);
      angularAccelerationInBody.set(tempAngularAccelerationInBody);
   }
   
   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   public void getOrientation(Matrix3d rotationMatrixToPack)
   {
      orientation.get(rotationMatrixToPack);
   }

   public void setAngularVelocityInBody(Vector3d angularVelocityInBody)
   {
      this.angularVelocityInBody.set(angularVelocityInBody);
   }
   
   public void getAngularVelocityInBody(Vector3d angularVelocityInBodyToPack)
   {
      angularVelocityInBody.get(angularVelocityInBodyToPack);
   }
   
   public void setAngularAccelerationInBody(Vector3d angularAccelerationInBody)
   {
      this.angularAccelerationInBody.set(angularAccelerationInBody);
   }
   
   public void getAngularAccelerationInBody(Vector3d angularAccelerationInBodyToPack)
   {
      angularAccelerationInBody.get(angularAccelerationInBodyToPack);
   }

   public void setLinearAccelerationInBody(Vector3d linearAccelerationInBody)
   {
      this.linearAccelerationInBody.set(linearAccelerationInBody);
   }
   
   public void getLinearAccelerationInBody(Vector3d linearAccelerationInBodyToPack)
   {
      linearAccelerationInBody.get(linearAccelerationInBodyToPack);
   }

   public void getTransformFromMountToJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformFromMountToJoint);
   }

   public void setOffset(RigidBodyTransform newTransformFromMountToJoint)
   {
      transformFromMountToJoint.set(newTransformFromMountToJoint);
   }

   
   public void setAngularVelocityNoiseParameters(double angularVelocityGaussianNoiseMean, double angularVelocityGaussianNoiseStdDev)
   {
      this.angularVelocityGaussianNoiseMean = angularVelocityGaussianNoiseMean;
      this.angularVelocityGaussianNoiseStdDev = angularVelocityGaussianNoiseStdDev;
   }
   
   public void setAngularVelocityBiasParameters(double angularVelocityGaussianBiasMean, double angularVelocityGaussianBiasStdDev)
   {
      this.angularVelocityGaussianBiasMean = angularVelocityGaussianBiasMean;
      this.angularVelocityGaussianBiasStdDev = angularVelocityGaussianBiasStdDev;
   }
   
   public void setAccelerationNoiseParameters(double accelerationGaussianNoiseMean, double accelerationGaussianNoiseStdDev)
   {
      this.accelerationGaussianNoiseMean = accelerationGaussianNoiseMean;
      this.accelerationGaussianNoiseStdDev = accelerationGaussianNoiseStdDev;
   }
   
   public void setAccelerationBiasParameters(double accelerationGaussianBiasMean, double accelerationGaussianBiasStdDev)
   {
      this.accelerationGaussianBiasMean = accelerationGaussianBiasMean;
      this.accelerationGaussianBiasStdDev = accelerationGaussianBiasStdDev;
   }

   public double getAccelerationGaussianNoiseMean()
   {
      return accelerationGaussianNoiseMean;
   }

   public double getAccelerationGaussianNoiseStdDev()
   {
      return accelerationGaussianNoiseStdDev;
   }

   public double getAccelerationGaussianBiasMean()
   {
      return accelerationGaussianBiasMean;
   }

   public double getAccelerationGaussianBiasStdDev()
   {
      return accelerationGaussianBiasStdDev;
   }

   public double getAngularVelocityGaussianNoiseMean()
   {
      return angularVelocityGaussianNoiseMean;
   }

   public double getAngularVelocityGaussianNoiseStdDev()
   {
      return angularVelocityGaussianNoiseStdDev;
   }

   public double getAngularVelocityGaussianBiasMean()
   {
      return angularVelocityGaussianBiasMean;
   }

   public double getAngularVelocityGaussianBiasStdDev()
   {
      return angularVelocityGaussianBiasStdDev;
   }
   
   
   
}
