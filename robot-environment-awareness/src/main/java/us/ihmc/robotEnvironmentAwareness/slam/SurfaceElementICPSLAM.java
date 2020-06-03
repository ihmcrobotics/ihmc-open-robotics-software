package us.ihmc.robotEnvironmentAwareness.slam;

import org.ejml.data.DenseMatrix64F;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;

public class SurfaceElementICPSLAM extends SLAMBasics
{
   public static final boolean DEBUG = false;
   
   private final AtomicDouble latestComputationTime = new AtomicDouble();

   public SurfaceElementICPSLAM(double octreeResolution)
   {
      super(octreeResolution);
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      long startTime = System.nanoTime();
      LogTools.info("instance of true");
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.05;
      int minimumNumberOfHits = 10;
      frame.registerSurfaceElements(octree, windowMargin, surfaceElementResolution, minimumNumberOfHits);

      int numberOfSurfel = frame.getSurfaceElementsToSensor().size();
      LogTools.info("numberOfSurfel " + numberOfSurfel);
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, numberOfSurfel);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = convertTransform(inputParameter.getData());
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame.getOriginalSensorPose());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame.getSurfaceElementsToSensor().get(i));

               correctedSensorPoseToWorld.transform(correctedSurfel[i].getPoint());
               correctedSensorPoseToWorld.transform(correctedSurfel[i].getNormal());
            }

            DenseMatrix64F errorSpace = new DenseMatrix64F(correctedSurfel.length, 1);
            for (int i = 0; i < correctedSurfel.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfel[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeSurfaceElementDistanceToNormalOctree(octree, surfel);
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.0001);
      purterbationVector.set(1, 0.0001);
      purterbationVector.set(2, 0.0001);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      // do ICP.
      for (int i = 0; i < 10; i++)
      {
         optimizer.iterate();
      }

      // get parameter.
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      icpTransformer.set(convertTransform(optimizer.getOptimalParameter().getData()));
//      LogTools.info("icpTransformer");
//      System.out.println(optimizer.getOptimalParameter());
//      System.out.println("Computation Time = " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
//      System.out.println(icpTransformer);
      latestComputationTime.set((double) Math.round(Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) * 100) / 100);
      return icpTransformer;
   }

   private RigidBodyTransform convertTransform(double... transformParameters)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(transformParameters[0], transformParameters[1], transformParameters[2]);
      transform.appendRollRotation(transformParameters[3]);
      transform.appendPitchRotation(transformParameters[4]);
      transform.appendYawRotation(transformParameters[5]);

      return transform;
   }
   
   public double getComputationTimeForLatestFrame()
   {
      return latestComputationTime.get();
   }
}
