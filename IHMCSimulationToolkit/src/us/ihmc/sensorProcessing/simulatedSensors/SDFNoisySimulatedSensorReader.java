package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class SDFNoisySimulatedSensorReader extends SDFPerfectSimulatedSensorReader
{

   private final boolean addNoiseFiltering = true;
   private double noiseFilterAlpha = 1e-1;
   private double quaternionNoiseStd = 0.01;
   private double positionNoiseStd = 0.01;

   private final Random rand = new Random(124381L);
   private final Quaternion rotationError = new Quaternion();
   private final Vector3D positionError = new Vector3D();
   private final Quaternion rotationFilter = new Quaternion();
   private final Vector3D positionFilter = new Vector3D();

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
      rotationError.set(rand.nextGaussian() * quaternionNoiseStd, rand.nextGaussian() * quaternionNoiseStd, rand.nextGaussian() * quaternionNoiseStd, 1);
      rotationError.normalize();

      positionError.setX(rand.nextGaussian() * positionNoiseStd);
      positionError.setY(rand.nextGaussian() * positionNoiseStd);
      positionError.setZ(rand.nextGaussian() * positionNoiseStd);

      RigidBodyTransform disturbanceTransform = new RigidBodyTransform();
      if (addNoiseFiltering)
      {
         double alpha = noiseFilterAlpha;
         // TODO reimplement me
//         rotationFilter.scale(1 - alpha);
//         rotationError.scale(alpha);
//         rotationFilter.add(rotationError);
//         rotationFilter.normalize();

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
