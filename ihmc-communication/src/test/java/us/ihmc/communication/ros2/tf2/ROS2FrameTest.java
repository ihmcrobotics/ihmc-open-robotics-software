package us.ihmc.communication.ros2.tf2;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.jros2.ROS2Node;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

public class ROS2FrameTest
{
   private static final double EPSILON = 1E-7;

   @Test
   public void testConstructingStaticFrame()
   {
      ROS2Node node = new ROS2Node("test_node");

      // Test basic static frame
      String id = "test_static_frame_0";
      ReferenceFrame worldFrame = ReferenceFrameTools.getWorldFrame();
      RigidBodyTransformReadOnly transformToParent = new RigidBodyTransform(new YawPitchRoll(0.1, 0.2, 0.3), new Vector3D(0.4, 0.5, 0.6));
      ROS2StaticFrame staticFrame = new ROS2StaticFrame(id, worldFrame, transformToParent);

      assertFalse(staticFrame.isWorldFrame());
      assertFalse(staticFrame.isRootFrame());
      assertFalse(staticFrame.isAStationaryFrame());
      assertFalse(staticFrame.isZupFrame());
      assertTrue(staticFrame.isFixedInParent());

      assertEquals(id, staticFrame.getFrameId());
      assertEquals(id, staticFrame.getName());
      assertEquals(worldFrame, staticFrame.getParent());
      EuclidCoreTestTools.assertEquals(transformToParent, staticFrame.getTransformToParent(), EPSILON);

      assertFalse(staticFrame.publishesMessageOnUpdate());

      // Test static frame that should publish its tf message every update
      ReferenceFrame grandParentFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("grandparent_frame",
                                                                                                        worldFrame,
                                                                                                        new RigidBodyTransform());
      ReferenceFrame parentFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("parent_frame",
                                                                                                       grandParentFrame,
                                                                                                       new RigidBodyTransform());
      String id2 = "test_static_frame_1";
      ROS2StaticFrame staticFrame1 = new ROS2StaticFrame(id2, parentFrame, new RigidBodyTransform());

      assertTrue(staticFrame1.publishesMessageOnUpdate());

      staticFrame.remove();
      staticFrame1.remove();
      node.destroy();
   }

   @Test
   public void testConstructingMutableFrame()
   {
      ROS2Node node = new ROS2Node("test_node");

      String id = "test_mutable_frame";
      ReferenceFrame parentFrame = ReferenceFrameTools.getWorldFrame();
      ROS2MutableFrame mutableFrame = new ROS2MutableFrame(id, parentFrame);

      assertFalse(mutableFrame.isWorldFrame());
      assertFalse(mutableFrame.isRootFrame());
      assertFalse(mutableFrame.isAStationaryFrame());
      assertFalse(mutableFrame.isZupFrame());
      assertFalse(mutableFrame.isFixedInParent());

      assertEquals(id, mutableFrame.getFrameId());
      assertEquals(id, mutableFrame.getName());
      assertEquals(parentFrame, mutableFrame.getParent());
      EuclidCoreTestTools.assertEquals(new RigidBodyTransform(), mutableFrame.getTransformToParent(), EPSILON);

      mutableFrame.remove();
      node.destroy();
   }

   @Test
   public void testUpdatingMutableFrame()
   {
      ROS2Node node = new ROS2Node("test_node");

      // We're going to run multiple update
      final int updatesToRun = 10;

      // Each update, this transform will be applied to the frame
      RigidBodyTransform updatedTransformToParent = new RigidBodyTransform(new YawPitchRoll(1.0, 2.0, 3.0), new Vector3D(4.0, 5.0, 6.0));

      // We'll also publish the updated transform, and ensure it's correct.
      AtomicInteger correctMessagesReceived = new AtomicInteger(0);
      node.createSubscription2(ROS2FrameTools.TF_TOPIC, message ->
      {
         synchronized (correctMessagesReceived)
         {
            RigidBodyTransform messageTransform = new RigidBodyTransform(message.getTransforms().get(0).getTransform());
            if (messageTransform.epsilonEquals(updatedTransformToParent, EPSILON))
               correctMessagesReceived.getAndIncrement();

            correctMessagesReceived.notify();
         }
      });

      // Construct a mutable frame
      String id = "test_mutable_frame";
      ReferenceFrame parentFrame = ReferenceFrameTools.getWorldFrame();
      ROS2MutableFrame mutableFrame = new ROS2MutableFrame(id, parentFrame);

      // Run updates
      for (int i = 0; i < updatesToRun; ++i)
      {
         updatedTransformToParent.appendTranslation(new Vector3D(0.1, 0.2, 0.3));
         updatedTransformToParent.appendOrientation(new YawPitchRoll(0.1, 0.2, 0.3));

         synchronized (correctMessagesReceived)
         {
            try
            {
               // Update the frame
               mutableFrame.setNewTransformToParent(updatedTransformToParent);
               mutableFrame.update();

               // Assert correct internal state
               EuclidCoreTestTools.assertEquals(updatedTransformToParent, mutableFrame.getTransformToParent(), EPSILON);

               // Publish a message and wait for it to be received
               if (correctMessagesReceived.get() != i + 1)
                  correctMessagesReceived.wait(1000);

               // Assert the message was correct
               assertEquals(i + 1, correctMessagesReceived.get());
            }
            catch (InterruptedException interruptedException)
            {
               throw new RuntimeException(interruptedException);
            }
         }
      }

      mutableFrame.remove();
      node.destroy();
   }

   @Test
   public void testPublishingMixedTree() throws InterruptedException
   {
      ROS2Node ros2Node = new ROS2Node("test_node");

      // Message counters to ensure correct messages
      AtomicInteger messagesReceived = new AtomicInteger(0);

      AtomicBoolean tfMessageReceived = new AtomicBoolean(false);
      AtomicInteger transformsInTFMessage = new AtomicInteger(0);

      AtomicBoolean tfStaticMessageReceived = new AtomicBoolean(false);
      AtomicInteger transformsInTFStaticMessage = new AtomicInteger(0);

      // Subscribe to /tf and /tf_static
      ros2Node.createSubscription2(ROS2FrameTools.TF_TOPIC, tfMessage ->
      {
         synchronized (messagesReceived)
         {
            messagesReceived.getAndIncrement();
            tfMessageReceived.set(true);
            transformsInTFMessage.set(tfMessage.getTransforms().size());

            messagesReceived.notify();
         }
      });

      ros2Node.createSubscription2(ROS2FrameTools.TF_STATIC_TOPIC, tfMessage ->
      {
         synchronized (messagesReceived)
         {
            messagesReceived.getAndIncrement();
            tfStaticMessageReceived.set(true);
            transformsInTFStaticMessage.set(tfMessage.getTransforms().size());

            messagesReceived.notify();
         }
      });

      // Construct a reference frame tree (world <- map <- odom <- base_link <- wrist <- gripper)
      ReferenceFrame worldFrame = ReferenceFrameTools.getWorldFrame();

      RigidBodyTransform mapToWorldTransform = new RigidBodyTransform(new YawPitchRoll(0.1, 0.2, 0.3), new Vector3D(0.4, 0.5, 0.6));
      ReferenceFrame mapFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("map", worldFrame, mapToWorldTransform);

      RigidBodyTransform odomToMapTransform = new RigidBodyTransform(new YawPitchRoll(0.7, 0.8, 0.9), new Vector3D(1.0, 1.1, 1.2));
      ReferenceFrame odomFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("odom", mapFrame, odomToMapTransform);

      RigidBodyTransform baseLinkToOdomTransform = new RigidBodyTransform(new YawPitchRoll(1.3, 1.4, 1.5), new Vector3D(1.6, 1.7, 1.8));
      ROS2MutableFrame ros2BaseLinkFrame = new ROS2MutableFrame("base_link", odomFrame, baseLinkToOdomTransform);

      RigidBodyTransform wristToBaseLinkTransform = new RigidBodyTransform(new YawPitchRoll(1.9, 2.0, 2.1), new Vector3D(2.2, 2.3, 2.4));
      ReferenceFrame wristFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("wrist", ros2BaseLinkFrame, wristToBaseLinkTransform);

      RigidBodyTransform wristToGripperTransform = new RigidBodyTransform(new YawPitchRoll(2.5, 2.6, 2.7), new Vector3D(2.8, 2.9, 3.0));
      ROS2StaticFrame ros2GripperFrame = new ROS2StaticFrame("gripper", wristFrame, wristToGripperTransform);

      for (int i = 0; i < 10; ++i)
      {
         // Update transforms
         odomToMapTransform.getTranslation().add(0.1, 0.1, 0.1);
         baseLinkToOdomTransform.getTranslation().add(0.2, 0.2, 0.2);
         ros2BaseLinkFrame.setNewTransformToParent(baseLinkToOdomTransform);
         wristToBaseLinkTransform.getTranslation().add(0.3, 0.3, 0.3);

         // Update reference frames
         mapFrame.update();
         odomFrame.update();

         synchronized (messagesReceived)
         {
            messagesReceived.set(0);
            tfMessageReceived.set(false);
            transformsInTFMessage.set(0);
            tfStaticMessageReceived.set(false);
            transformsInTFStaticMessage.set(0);

            ros2BaseLinkFrame.update();

            while (messagesReceived.get() < 2)
               messagesReceived.wait();

            assertTrue(tfMessageReceived.get());
            assertEquals(2, transformsInTFMessage.get());

            assertTrue(tfStaticMessageReceived.get());
            assertEquals(1, transformsInTFStaticMessage.get());
         }

         wristFrame.update();

         synchronized (messagesReceived)
         {
            messagesReceived.set(0);
            tfMessageReceived.set(false);
            transformsInTFMessage.set(0);
            tfStaticMessageReceived.set(false);
            transformsInTFStaticMessage.set(0);

            ros2GripperFrame.update();

            while (messagesReceived.get() < 2)
               messagesReceived.wait();

            assertTrue(tfMessageReceived.get());
            assertEquals(1, transformsInTFMessage.get());

            assertTrue(tfStaticMessageReceived.get());
            assertEquals(1, transformsInTFStaticMessage.get());
         }
      }
   }
}
