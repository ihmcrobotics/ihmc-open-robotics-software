package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;

public class SurfaceElementICPSLAMFrame extends SLAMFrame
{
   private final List<Plane3D> surfaceElements = new ArrayList<>();
   private final List<Plane3DReadOnly> surfaceElementsToSensor = new ArrayList<>();

   public SurfaceElementICPSLAMFrame(StereoVisionPointCloudMessage message)
   {
      super(message);
   }

   public SurfaceElementICPSLAMFrame(SLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      super(frame, message);
   }

   public void registerSurfaceElements(NormalOcTree map, double windowMargin, double surfaceElementResolution, int minimumNumberOfHits)
   {
      surfaceElements.clear();
      surfaceElementsToSensor.clear();
      NormalOcTree frameMap = new NormalOcTree(surfaceElementResolution);

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = getOriginalPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(getOriginalPointCloud(), getOriginalPointCloudToSensorPose(), getOriginalSensorPose(), map, windowMargin));

      frameMap.insertScanCollection(scanCollection, false);
      frameMap.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      normalEstimationParameters.setMaxDistanceFromPlane(surfaceElementResolution);
      frameMap.setNormalEstimationParameters(normalEstimationParameters);
      frameMap.updateNormals();

      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(frameMap.getRoot());
      for (NormalOcTreeNode node : iterable)
      {
         if (node.getNumberOfHits() >= minimumNumberOfHits)
         {
            if (node.getNormalAverageDeviation() < 0.00005)
            {
               Plane3D surfaceElement = new Plane3D();
               node.getNormal(surfaceElement.getNormal());
               node.getHitLocation(surfaceElement.getPoint());

               surfaceElements.add(surfaceElement);
               surfaceElementsToSensor.add(SLAMTools.createConvertedSurfaceElementToSensorPose(getOriginalSensorPose(), surfaceElement));
            }
         }
      }
      LogTools.info("surface elements[" + surfaceElements.size() + "].");
   }

   @Override
   public void updateOptimizedCorrection(RigidBodyTransformReadOnly driftCorrectionTransform)
   {
      super.updateOptimizedCorrection(driftCorrectionTransform);

      for (int i = 0; i < surfaceElements.size(); i++)
      {
         Plane3D surfel = surfaceElements.get(i);
         surfel.set(surfaceElementsToSensor.get(i));
         getSensorPose().transform(surfel.getPoint());
         getSensorPose().transform(surfel.getNormal());
      }
   }

   public List<Plane3D> getSurfaceElements()
   {
      return surfaceElements;
   }

   public List<Plane3DReadOnly> getSurfaceElementsToSensor()
   {
      return surfaceElementsToSensor;
   }
}
