package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.UnaryOperator;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
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
      SurfaceElementICPSLAMParameters parameters = this.parameters.getAndSet(null);
      if (parameters == null)
         return null;

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
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(octree.getResolution() * 1.5);

      // do ICP.
      for (int i = 0; i < 20; i++)
      {
         optimizer.iterate();
         // TODO: add terminal condition. 

      }

      // get parameter.
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      optimizer.convertInputToTransform(optimizer.getOptimalParameter(), icpTransformer);

      return icpTransformer;
   }
}
