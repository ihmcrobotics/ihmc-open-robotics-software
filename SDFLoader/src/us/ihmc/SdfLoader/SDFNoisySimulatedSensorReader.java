package us.ihmc.SdfLoader;

import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

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

   public SDFNoisySimulatedSensorReader(SDFRobot robot, FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames)
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
   protected void packRootTransform(SDFRobot robot, Transform3D transformToPack)
   {
      super.packRootTransform(robot, transformToPack);
      rotationError.w = 1;
      rotationError.x = rand.nextGaussian() * quaternionNoiseStd;
      rotationError.y = rand.nextGaussian() * quaternionNoiseStd;
      rotationError.z = rand.nextGaussian() * quaternionNoiseStd;
      rotationError.normalize();

      positionError.x = rand.nextGaussian() * positionNoiseStd;
      positionError.y = rand.nextGaussian() * positionNoiseStd;
      positionError.z = rand.nextGaussian() * positionNoiseStd;

      Transform3D disturbanceTransform = new Transform3D();
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
         disturbanceTransform.set(rotationFilter, positionFilter, 1.0);
      }
      else
      {
         disturbanceTransform.set(rotationError, positionError, 1.0);
      }

      transformToPack.mul(disturbanceTransform);

   }

}
