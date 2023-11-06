package us.ihmc.perception.sceneGraph;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class SceneGraphProcess
{
   private final ROS2Helper ros2Helper;
   private final ROS2SceneGraph sceneGraph;

   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private BytedecoImage arUcoBytedecoImage;
   private RawImage arUcoImage;
   private Supplier<ReferenceFrame> blackflyFrameSupplier;
   private long lastArUcoImageSequenceNumber = -1L;

   private final RestartableThread arUcoDetectionThread;
   private final Lock arUcoLock = new ReentrantLock();
   private final Condition arUcoUpdateCondition = arUcoLock.newCondition();

   public SceneGraphProcess()
   {
      ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "scene_graph_node");
      ros2Helper = new ROS2Helper(node);
      sceneGraph = new ROS2SceneGraph(ros2Helper);

      arUcoDetectionThread = new RestartableThread("ArUcoMarkerDetector", this::updateArUcoDetection);
   }

   public void initializeArUcoProcess(int imageWidth, int imageHeight, Supplier<ReferenceFrame> blackflyFrameSupplier, Mat cameraMatrixEstimate)
   {
      this.blackflyFrameSupplier = blackflyFrameSupplier;
      arUcoBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(blackflyFrameSupplier.get());
      arUcoMarkerDetection.setSourceImageForDetection(arUcoBytedecoImage);
      cameraMatrixEstimate.copyTo(arUcoMarkerDetection.getCameraMatrix());
      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection, ros2Helper, sceneGraph.getArUcoMarkerIDToNodeMap());
   }

   private void updateArUcoDetection()
   {
      arUcoLock.lock();
      try
      {
         while ((arUcoImage == null || arUcoImage.isEmpty() || arUcoImage.getSequenceNumber() == lastArUcoImageSequenceNumber)
                && arUcoDetectionThread.isRunning())
         {
            arUcoUpdateCondition.await();
         }

         arUcoImage.getCpuImageMatrix().copyTo(arUcoBytedecoImage.getBytedecoOpenCVMat());
         lastArUcoImageSequenceNumber = arUcoImage.getSequenceNumber();

         arUcoMarkerDetection.update();
         arUcoMarkerPublisher.update();
         ArUcoSceneTools.updateSceneGraph(arUcoMarkerDetection, sceneGraph);

//         sceneGraph.updateSubscription();
         sceneGraph.updateOnRobotOnly(blackflyFrameSupplier.get());
         //         sceneGraph.updatePublication();
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException.getMessage());
      }
      finally
      {
         arUcoLock.unlock();
      }
   }

   public void startArUcoDetection()
   {
      arUcoDetectionThread.start();
   }

   public void stopArUcoDetection()
   {
      arUcoDetectionThread.stop();
      arUcoLock.lock();
      try
      {
         arUcoUpdateCondition.signal();
      }
      finally
      {
         arUcoLock.unlock();
      }
   }

   public void destroy()
   {
      arUcoLock.lock();
      try
      {
         arUcoUpdateCondition.signal();
      }
      finally
      {
         arUcoLock.unlock();
      }

      arUcoDetectionThread.blockingStop();

      arUcoImage.release();
      arUcoMarkerDetection.destroy();
   }

   public void setNextArUcoImage(RawImage undistortedArUcoImage)
   {
      arUcoLock.lock();
      try
      {
         if (arUcoImage != null)
            arUcoImage.release();
         arUcoImage = undistortedArUcoImage;
         arUcoUpdateCondition.signal();
      }
      finally
      {
         arUcoLock.unlock();
      }
   }
}
