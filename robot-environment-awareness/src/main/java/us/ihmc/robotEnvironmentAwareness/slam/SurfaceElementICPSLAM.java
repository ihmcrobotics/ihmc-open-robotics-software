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
   private final AtomicReference<SurfaceElementICPSLAMParameters> parameters = new AtomicReference<>(new SurfaceElementICPSLAMParameters());

   public SurfaceElementICPSLAM(double octreeResolution)
   {
      super(octreeResolution);
   }

   public void updateParameters(SurfaceElementICPSLAMParameters parameters)
   {
      this.parameters.set(parameters);
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      SurfaceElementICPSLAMParameters surfaceElementICPSLAMParameters = parameters.get();
      Function<DMatrixRMaj, RigidBodyTransform> transformConverter = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(surfaceElementICPSLAMParameters.getIncludePitchAndRoll());

      double surfaceElementResolution = surfaceElementICPSLAMParameters.getSurfaceElementResolution();
      double windowMargin = surfaceElementICPSLAMParameters.getWindowMargin();
      int minimumNumberOfHits = surfaceElementICPSLAMParameters.getMinimumNumberOfHit();
      frame.registerSurfaceElements(getOctree(), windowMargin, surfaceElementResolution, minimumNumberOfHits, surfaceElementICPSLAMParameters.getComputeSurfaceNormalsInFrame());

      int numberOfSurfel = frame.getSurfaceElementsInSensorFrame().size();
      sourcePoints = new Point3D[numberOfSurfel];
      if (DEBUG)
         LogTools.info("numberOfSurfel " + numberOfSurfel);
      UnaryOperator<DMatrixRMaj> outputCalculator = new UnaryOperator<DMatrixRMaj>()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform(transformConverter.apply(inputParameter));
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame.getUncorrectedSensorPoseInWorld());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame.getSurfaceElementsInSensorFrame().get(i));

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
            return SLAMTools.computeBoundedPerpendicularDistancePointToNormalOctree(octree,
                                                                                    surfel.getPoint(),
                                                                                    octree.getResolution() * surfaceElementICPSLAMParameters.getBoundRatio());
         }
      };
      int problemSize = surfaceElementICPSLAMParameters.getIncludePitchAndRoll() ? 6 : 4;
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(transformConverter, outputCalculator, problemSize, numberOfSurfel);
      DMatrixRMaj perturbationVector = new DMatrixRMaj(problemSize, 1);
      double translationPerturbation = octree.getResolution() * surfaceElementICPSLAMParameters.getTranslationPerturbation();
      perturbationVector.set(0, translationPerturbation);
      perturbationVector.set(1, translationPerturbation);
      perturbationVector.set(2, translationPerturbation);
      perturbationVector.set(3, surfaceElementICPSLAMParameters.getRotationPerturbation());
      if (surfaceElementICPSLAMParameters.getIncludePitchAndRoll())
      {
         perturbationVector.set(4, surfaceElementICPSLAMParameters.getRotationPerturbation());
         perturbationVector.set(5, surfaceElementICPSLAMParameters.getRotationPerturbation());
      }
      optimizer.setPerturbationVector(perturbationVector);
      boolean initialCondition = optimizer.initialize();
      if (DEBUG)
         LogTools.info("initialCondition " + initialCondition);
      if (!initialCondition)
      {
         //TODO: Ticket, FB-638.
         //return null;
      }
      optimizer.setCorrespondenceThreshold(surfaceElementICPSLAMParameters.getMinimumCorrespondingDistance()); // Note : (x 1.5) of surfel resolution.
      double initialQuality = optimizer.getQuality();
      driftCorrectionResult.setInitialDistance(initialQuality);
      if (DEBUG)
         LogTools.info("initial quality " + initialQuality);

      if (surfaceElementICPSLAMParameters.isEnableInitialQualityFilter())
      {
         if (initialQuality > surfaceElementICPSLAMParameters.getInitialQualityThreshold())
         {
            LogTools.warn("initial quality is very bad ! " + initialQuality);
            return null;
         }
      }

      double qualitySteadyThreshold = surfaceElementICPSLAMParameters.getQualityConvergenceThreshold();
      double translationalSteadyThreshold = surfaceElementICPSLAMParameters.getTranslationalEffortConvergenceThreshold();
      double rotationalSteadyThreshold = surfaceElementICPSLAMParameters.getRotationalEffortConvergenceThreshold();

      SteadyDetector qualitySteady = new SteadyDetector(0.0, qualitySteadyThreshold);
      SteadyDetector translationalSteady = new SteadyDetector(0.0, translationalSteadyThreshold);
      RotationalEffortSteadyDetector rotationalSteady = new RotationalEffortSteadyDetector(rotationalSteadyThreshold);

      int numberOfSteadyIterations = 0;
      int steadyIterationsThreshold = surfaceElementICPSLAMParameters.getSteadyStateDetectorIterationThreshold();
      // do ICP.
      double quality = 0.0;
      double translationalEffort = 0.0;
      RotationMatrix rotationalEffort = new RotationMatrix();
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      int iterations = -1;
      for (int i = 0; i < surfaceElementICPSLAMParameters.getMaxOptimizationIterations(); i++)
      {
         optimizer.iterate();
         optimizer.convertInputToTransform(optimizer.getOptimalParameter(), icpTransformer);

         quality = initialQuality;
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
            if (DEBUG)
               LogTools.info("################ " + i);
            iterations = i;
            break;
         }
      }

      if (DEBUG)
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
         return Math.abs(value - other);
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
