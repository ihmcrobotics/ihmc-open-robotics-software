package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;

public class SLAMBasics implements SLAMInterface
{
   private final AtomicReference<SLAMFrame> latestSlamFrame = new AtomicReference<>(null);
   protected final NormalOcTree octree;
   private final AtomicInteger mapSize = new AtomicInteger();

   private final AtomicDouble latestComputationTime = new AtomicDouble();

   public SLAMBasics(double octreeResolution)
   {
      octree = new NormalOcTree(octreeResolution);
   }

   protected void insertNewPointCloud(SLAMFrame frame)
   {
      Point3DReadOnly[] pointCloud = frame.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = frame.getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation()));

      octree.insertScanCollection(scanCollection, true);
      octree.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      octree.setNormalEstimationParameters(normalEstimationParameters);
   }

   public void updatePlanarRegionsMap()
   {
      octree.updateNormals();
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      SLAMFrame frame = new SLAMFrame(pointCloudMessage);
      setLatestFrame(frame);
      insertNewPointCloud(frame);
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      SLAMFrame frame = new SLAMFrame(getLatestFrame(), pointCloudMessage);

      long startTime = System.nanoTime();
      RigidBodyTransformReadOnly optimizedMultiplier = computeFrameCorrectionTransformer(frame);
      latestComputationTime.set((double) Math.round(Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) * 100) / 100);

      if (optimizedMultiplier == null)
      {
         return false;
      }
      else
      {
         frame.updateOptimizedCorrection(optimizedMultiplier);
         setLatestFrame(frame);
         insertNewPointCloud(frame);

         return true;
      }
   }

   @Override
   public void clear()
   {
      latestSlamFrame.set(null);
      mapSize.set(0);
      octree.clear();
   }

   public boolean isEmpty()
   {
      if (latestSlamFrame.get() == null)
         return true;
      else
         return false;
   }

   public int getMapSize()
   {
      return mapSize.get();
   }

   public void setLatestFrame(SLAMFrame frameToSet)
   {
      latestSlamFrame.set(frameToSet);
      mapSize.incrementAndGet();
   }

   public SLAMFrame getLatestFrame()
   {
      return latestSlamFrame.get();
   }

   public double getOctreeResolution()
   {
      return octree.getResolution();
   }

   public NormalOcTree getOctree()
   {
      return octree;
   }

   public double getComputationTimeForLatestFrame()
   {
      return latestComputationTime.get();
   }
}
