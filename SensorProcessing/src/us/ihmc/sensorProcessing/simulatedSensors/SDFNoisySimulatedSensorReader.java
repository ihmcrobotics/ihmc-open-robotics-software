package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class SDFNoisySimulatedSensorReader extends SDFPerfectSimulatedSensorReader
{

   private final boolean addNoiseFiltering = true;
   private double noiseFilterAlpha = 1e-1;
   private double quaternionNoiseStd = 0.01;
   private double positionNoiseStd = 0.01;

   private final Random rand = new Random(124381L);
   private final Quat4d rotationError = new Quat4d();
   private final Vector3d positionError = new Vector3d();
   private final Quat4d rotationFilter = new Quat4d();
   private final Vector3d positionFilter = new Vector3d();

   public SDFNoisySimulatedSensorReader(FloatingRootJointRobot robot, FullRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames)
   {
      super(robot, fullRobotModel, referenceFrames);
   }

   public void setNoiseFilterAlpha(double noiseFilterAlpha)
   {
      this.noiseFilterAlpha = noiseFilterAlpha;
   }

   public void setQuaternionNoiseStd(double quaternionNoiseStd)
   {
      this.quaternionNoiseStd = quaternionNoiseStd;
   }

   public void setPositionNoiseStd(double positionNoiseStd)
   {
      this.positionNoiseStd = positionNoiseStd;
   }

   @Override
   protected void packRootTransform(FloatingRootJointRobot robot, RigidBodyTransform transformToPack)
   {
      super.packRootTransform(robot, transformToPack);
      rotationError.setW(1);
      rotationError.setX(rand.nextGaussian() * quaternionNoiseStd);
      rotationError.setY(rand.nextGaussian() * quaternionNoiseStd);
      rotationError.setZ(rand.nextGaussian() * quaternionNoiseStd);
      rotationError.normalize();

      positionError.setX(rand.nextGaussian() * positionNoiseStd);
      positionError.setY(rand.nextGaussian() * positionNoiseStd);
      positionError.setZ(rand.nextGaussian() * positionNoiseStd);

      RigidBodyTransform disturbanceTransform = new RigidBodyTransform();
      if (addNoiseFiltering)
      {
         double alpha = noiseFilterAlpha;
         rotationFilter.scale(1 - alpha);
         rotationError.scale(alpha);
         rotationFilter.add(rotationError);
         rotationFilter.normalize();

         positionFilter.scale(1 - alpha);
         positionError.scale(alpha);
         positionFilter.add(positionError);
         disturbanceTransform.set(rotationFilter, positionFilter);
      }
      else
      {
         disturbanceTransform.set(rotationError, positionError);
      }

      transformToPack.multiply(disturbanceTransform);

   }

}
