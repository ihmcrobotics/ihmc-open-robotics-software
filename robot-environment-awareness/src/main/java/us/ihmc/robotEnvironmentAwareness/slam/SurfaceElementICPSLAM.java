package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.UnaryOperator;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;

public class SurfaceElementICPSLAM extends SLAMBasics
{
   public static final boolean DEBUG = false;
   private final Function<DMatrixRMaj, RigidBodyTransform> transformConverter = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction();

   private final AtomicReference<SurfaceElementICPSLAMParameters> parameters = new AtomicReference<SurfaceElementICPSLAMParameters>(new SurfaceElementICPSLAMParameters());

   public SurfaceElementICPSLAM(double octreeResolution)
   {
      super(octreeResolution);
   }

   public void updateParameters(SurfaceElementICPSLAMParameters parameters)
   {
      this.parameters.set(parameters);
   }

   /**
    * see {@link SurfaceElementICPBasedDriftCorrectionVisualizer}
    */
   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
//      SurfaceElementICPSLAMParameters parameters = this.parameters.getAndSet(null);
//      if (parameters == null)
//         return null;

      // TODO: add diagnosing manager.
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.0;
      int minimumNumberOfHits = 1;
      frame.registerSurfaceElements(octree, windowMargin, surfaceElementResolution, minimumNumberOfHits, true);

      int numberOfSurfel = frame.getSurfaceElementsToSensor().size();
      sourcePoints = new Point3D[numberOfSurfel];
      LogTools.info("numberOfSurfel " + numberOfSurfel);
      UnaryOperator<DMatrixRMaj> outputCalculator = new UnaryOperator<DMatrixRMaj>()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform(transformConverter.apply(inputParameter));
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame.getOriginalSensorPose());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame.getSurfaceElementsToSensor().get(i));

               correctedSensorPoseToWorld.transform(correctedSurfel[i].getPoint());
               correctedSensorPoseToWorld.transform(correctedSurfel[i].getNormal());
               sourcePoints[i] = new Point3D(correctedSensorPoseToWorld.getTranslation());
            }

            DMatrixRMaj errorSpace = new DMatrixRMaj(correctedSurfel.length, 1);
            for (int i = 0; i < correctedSurfel.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfel[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeBoundedPerpendicularDistancePointToNormalOctree(octree, surfel.getPoint(), octree.getResolution() * 1.1);
         }
      };
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(transformConverter, outputCalculator, 6, numberOfSurfel);
      DMatrixRMaj purterbationVector = new DMatrixRMaj(6, 1);
      purterbationVector.set(0, octree.getResolution() * 0.002);
      purterbationVector.set(1, octree.getResolution() * 0.002);
      purterbationVector.set(2, octree.getResolution() * 0.002);
      purterbationVector.set(3, 0.00001);
      purterbationVector.set(4, 0.00001);
      purterbationVector.set(5, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      boolean initialCondition = optimizer.initialize();
      LogTools.info("initialCondition " + initialCondition);
      optimizer.setCorrespondenceThreshold(octree.getResolution() * 1.5);
      driftCorrectionResult.setInitialDistance(optimizer.getQuality());
      LogTools.info("initial quality " + optimizer.getQuality());

      double qualitySteadyThreshold = 0.001;
      double translationalSteadyThreshold = 0.001;
      double rotationalSteadyThreshold = 0.005;

      SteadyDetector qualitySteady = new SteadyDetector(0.0, qualitySteadyThreshold);
      SteadyDetector translationalSteady = new SteadyDetector(0.0, translationalSteadyThreshold);
      RotationalEffortSteadyDetector rotationalSteady = new RotationalEffortSteadyDetector(rotationalSteadyThreshold);

      int numberOfSteadyIterations = 0;
      int steadyIterationsThreshold = 3;
      // do ICP.
      double quality = 0.0;
      double translationalEffort = 0.0;
      RotationMatrix rotationalEffort = new RotationMatrix();
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      int iterations = -1;
      for (int i = 0; i < 40; i++)
      {
         optimizer.iterate();
         optimizer.convertInputToTransform(optimizer.getOptimalParameter(), icpTransformer);

         quality = optimizer.getQuality();
         translationalEffort = icpTransformer.getTranslation().lengthSquared();
         rotationalEffort.set(icpTransformer.getRotation());

         if (qualitySteady.isSteady(quality) && translationalSteady.isSteady(translationalEffort) && rotationalSteady.isSteady(rotationalEffort))
         {
            numberOfSteadyIterations++;
         }
         else
         {
            numberOfSteadyIterations = 0;
         }

         if (numberOfSteadyIterations >= steadyIterationsThreshold)
         {
            LogTools.info("################ " + i);
            iterations = i;
            break;
         }
      }

      LogTools.info("final quality " + optimizer.getQuality());
      // get parameter.
      optimizer.convertInputToTransform(optimizer.getOptimalParameter(), icpTransformer);

      driftCorrectionResult.setFinalDistance(optimizer.getQuality());
      driftCorrectionResult.setNumberOfSurfels(numberOfSurfel);
      driftCorrectionResult.setDriftCorrectionTransformer(icpTransformer);
      driftCorrectionResult.setIcpIterations(iterations);
      if (iterations < 0)
      {
         driftCorrectionResult.setSuccess(false);
      }
      else
      {
         driftCorrectionResult.setSuccess(true);
      }

      return icpTransformer;
   }

   private class SteadyDetector
   {
      private double previous;
      private final double threshold;

      SteadyDetector(double current, double threshold)
      {
         previous = current;
         this.threshold = threshold;
      }

      boolean isSteady(double current)
      {
         boolean isSteady = distance(previous, current) < threshold;
         previous = current;
         return isSteady;
      }

      double distance(double value, double other)
      {
         return Math.abs(previous - other);
      }
   }

   private class RotationalEffortSteadyDetector
   {
      private final RotationMatrix previous = new RotationMatrix();
      private final double threshold;

      RotationalEffortSteadyDetector(double threshold)
      {
         this.threshold = threshold;
      }

      boolean isSteady(RotationMatrix current)
      {
         boolean isSteady = distance(previous, current) < threshold;
         previous.set(current);
         return isSteady;
      }

      double distance(RotationMatrix value, RotationMatrix other)
      {
         return new Quaternion(value).distance(new Quaternion(other));
      }
   }
}
