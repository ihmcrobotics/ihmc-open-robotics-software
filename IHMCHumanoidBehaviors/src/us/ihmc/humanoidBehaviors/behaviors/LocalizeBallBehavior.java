package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import georegression.struct.shapes.Sphere3D_F64;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.utilities.BallPoseEstimator;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class LocalizeBallBehavior extends BehaviorInterface
{

   private BooleanYoVariable ballFound = new BooleanYoVariable("ballFound", registry);
   private DoubleYoVariable ballRadius = new DoubleYoVariable("ballRadius", registry);
   private DoubleYoVariable ballX = new DoubleYoVariable("ballX", registry);
   private DoubleYoVariable ballY = new DoubleYoVariable("ballY", registry);
   private DoubleYoVariable ballZ = new DoubleYoVariable("ballZ", registry);
   private DoubleYoVariable totalBallsFound = new DoubleYoVariable("totalBallsFound", registry);
   private DoubleYoVariable smallestBallFound = new DoubleYoVariable("smallestBallFound", registry);


   private final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>();

   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final BallPoseEstimator ballPoseEstimator;

   public LocalizeBallBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames)
   {
      super(outgoingCommunicationBridge);
      this.attachNetworkProcessorListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);
      this.humanoidReferenceFrames = referenceFrames;
      ballPoseEstimator = new BallPoseEstimator();
   }

   public boolean foundBall()
   {
      return ballFound.getBooleanValue();
   }

   public void reset()
   {
      ballFound.set(false);
   }

   public Point3d getBallLocation()
   {
      return new Point3d(ballX.getDoubleValue(), ballY.getDoubleValue(), ballZ.getDoubleValue());
   }

   PointCloudWorldPacket pointCloudPacket;
   PointCloudWorldPacket pointCloudPacketLatest = null;

   @Override
   public void doControl()
   {

      while ((pointCloudPacket = pointCloudQueue.getNewestPacket()) != null)
      {
         pointCloudPacketLatest = pointCloudPacket;
      }

      if (pointCloudPacketLatest != null)
      {
         Point3f[] points = pointCloudPacketLatest.getDecayingWorldScan();
         findBallsAndSaveResult(points);
      }

   }

   private void findBallsAndSaveResult(Point3f[] points)
   {
      ArrayList<Sphere3D_F64> balls = ballPoseEstimator.detectBalls(points);

      totalBallsFound.set(ballPoseEstimator.getNumberOfBallsFound());
      smallestBallFound.set(ballPoseEstimator.getSmallestRadius());


      for (Sphere3D_F64 ball : balls)
      {
         RigidBodyTransform t = new RigidBodyTransform();
         t.setTranslation(ball.getCenter().x, ball.getCenter().y, ball.getCenter().z);
         sendPacketToNetworkProcessor(new DetectedObjectPacket(t, 4));
      }


      if (balls.size() > 0)
      {
         ballFound.set(true);
         ballRadius.set(balls.get(0).radius);
         ballX.set(balls.get(0).getCenter().x);
         ballY.set(balls.get(0).getCenter().y);
         ballZ.set(balls.get(0).getCenter().z);
      }
      else
      {
         ballFound.set(false);
         ballRadius.set(0);
         ballX.set(0);
         ballY.set(0);
         ballZ.set(0);
      }

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
      return ballFound.getBooleanValue();
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
      return true;
   }

   @Override
   public void initialize()
   {
      defaultPostBehaviorCleanup();
   }
}
