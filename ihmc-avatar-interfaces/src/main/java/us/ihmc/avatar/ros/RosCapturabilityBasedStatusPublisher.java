package us.ihmc.avatar.ros;

import java.util.concurrent.ArrayBlockingQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosBoolPublisher;
import us.ihmc.utilities.ros.publisher.RosPoint2dPublisher;
import us.ihmc.utilities.ros.publisher.RosPoint32Publisher;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosCapturabilityBasedStatusPublisher implements PacketConsumer<CapturabilityBasedStatus>, Runnable
{
   private final ArrayBlockingQueue<CapturabilityBasedStatus> availableCapturabilityStatus = new ArrayBlockingQueue<>(30);

   private final RosPoint2dPublisher capturePointPublisher = new RosPoint2dPublisher(false);
   private final RosPoint2dPublisher desiredCapturePointPublisher = new RosPoint2dPublisher(false);
   private final RosPoint32Publisher centerOfMassPublisher = new RosPoint32Publisher(false);
   
   private final RosBoolPublisher isInDoubleSupportPublisher = new RosBoolPublisher(false);

   private final SideDependentList<RosSupportPolygonPublisher> supportPolygonPublishers = new SideDependentList<>();
   private final RosMainNode rosMainNode;

   public RosCapturabilityBasedStatusPublisher(RosMainNode rosMainNode, String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;

      for (RobotSide value : RobotSide.values)
      {
         RosSupportPolygonPublisher rosSupportPolygonPublisher = new RosSupportPolygonPublisher(false);
         supportPolygonPublishers.put(value, rosSupportPolygonPublisher);
         this.rosMainNode
               .attachPublisher(rosNameSpace + "/output/capturability/" + value.getCamelCaseNameForStartOfExpression() + "_foot_support_polygon", rosSupportPolygonPublisher);
      }

      this.rosMainNode.attachPublisher(rosNameSpace + "/output/capturability/capture_point", capturePointPublisher);
      this.rosMainNode.attachPublisher(rosNameSpace + "/output/capturability/desired_capture_point", desiredCapturePointPublisher);
      this.rosMainNode.attachPublisher(rosNameSpace + "/output/capturability/center_of_mass", centerOfMassPublisher);
      this.rosMainNode.attachPublisher(rosNameSpace + "/output/capturability/is_in_double_support", isInDoubleSupportPublisher);

      new Thread(this, getClass().getName()).start();
   }

   @Override
   public void receivedPacket(CapturabilityBasedStatus packet)
   {
      if (!availableCapturabilityStatus.offer(packet))
      {
         availableCapturabilityStatus.clear();
      }
   }

   @Override
   public void run()
   {
      while (true)
      {
         CapturabilityBasedStatus capturabilityBasedStatus;
         try
         {
            capturabilityBasedStatus = availableCapturabilityStatus.take();
         }
         catch (InterruptedException e)
         {
            continue;
         }

         if (rosMainNode.isStarted())
         {
            capturePointPublisher.publish(capturabilityBasedStatus.capturePoint);
            desiredCapturePointPublisher.publish(capturabilityBasedStatus.desiredCapturePoint);
            centerOfMassPublisher.publish(capturabilityBasedStatus.centerOfMass);
            isInDoubleSupportPublisher.publish(capturabilityBasedStatus.isInDoubleSupport());

//            for (RobotSide value : RobotSide.values)
//            {
//               RosSupportPolygonPublisher rosSupportPolygonPublisher = supportPolygonPublishers.get(value);
//               switch (value)
//               {
//               case LEFT:
//                  rosSupportPolygonPublisher.publish(capturabilityBasedStatus.leftFootSupportPolygonStore, capturabilityBasedStatus.leftFootSupportPolygonLength);
//                  break;
//               case RIGHT:
//                  rosSupportPolygonPublisher.publish(capturabilityBasedStatus.rightFootSupportPolygonStore, capturabilityBasedStatus.rightFootSupportPolygonLength);
//                  break;
//               }
//            }
         }
      }
   }
}
