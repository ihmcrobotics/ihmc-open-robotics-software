package us.ihmc.humanoidBehaviors.behaviors;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;

import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import boofcv.struct.calib.IntrinsicParameters;

public class LocalizeDrillBehavior extends BehaviorInterface implements VideoStreamer
{

   private LongYoVariable counter = new LongYoVariable("counter", registry);

   private final ConcurrentListeningQueue<VideoPacket> videoQueue = new ConcurrentListeningQueue<VideoPacket>();
   private CompressedVideoDataClient compressedVideoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);

   private final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>();

   private final HumanoidReferenceFrames humanoidReferenceFrames;

   public LocalizeDrillBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames)
   {
      super(outgoingCommunicationBridge);
      this.attachNetworkProcessorListeningQueue(videoQueue, VideoPacket.class);
      this.attachNetworkProcessorListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);
      this.humanoidReferenceFrames = referenceFrames;
   }

   private void setupUI()
   {
      //DRCSpriteWorldGenerator::generateColors
   }

   @Override
   public void doControl()
   {
      VideoPacket videoPacket;

      while ((videoPacket = videoQueue.getNewestPacket()) != null)
      {
         //still empty the incoming queue
         if (!isPaused())
         {
            compressedVideoDataClient.consumeObject(videoPacket.data, videoPacket.position, videoPacket.orientation, videoPacket.intrinsicParameters);
         }
      }

      PointCloudWorldPacket pointCloudPacket;
      while ((pointCloudPacket = pointCloudQueue.getNewestPacket()) != null)
      {
         //still empty the incoming queue
         if (!isPaused())
         {
            Point3f[] points = pointCloudPacket.getDecayingWorldScan();
            findDrillAndSendResult(points);
         }
      }
   }

   private void findDrillAndSendResult(Point3f[] points)
   {
      System.out.println("got " + points.length);

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      Point3d pelvisInWorld = new Point3d();
      pelvisZUpFrame.getTransformToWorldFrame().getTranslation(pelvisInWorld);
      RigidBodyTransform transformFromWorld = new RigidBodyTransform();
      transformFromWorld.invert(pelvisZUpFrame.getTransformToWorldFrame());
      
      Point3f[] pointsInPelvisFrame = transformPoints(points, transformFromWorld);
      saveCloud(pointsInPelvisFrame);

   }
   
   private Point3f[] transformPoints(Point3f[] srcPoints, RigidBodyTransform transform)
   {
            Point3f[] dstPoints = new Point3f[srcPoints.length];
            for(int i=0;i<srcPoints.length;i++)
            {
               Point3f dstPoint = new Point3f();
               transform.transform(srcPoints[i], dstPoint);
               dstPoints[i] = dstPoint;
            }
            return dstPoints;
   }
   
   private void saveCloud(Point3f[] points)
   {
      PrintStream output;
      try
      {
         output = new PrintStream(new File("/tmp/pointcloud.tx"));
         for (int i = 0; i < points.length; i++)
         {
            Point3f point = points[i];

            output.println(point.x + " " + point.y + " " + point.z);
         }
         output.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters)
   {
      counter.increment();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   public boolean isDone()
   {
      return defaultIsDone();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void initialize()
   {
      defaultPostBehaviorCleanup();
   }
}
