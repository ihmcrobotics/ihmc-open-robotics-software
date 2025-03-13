package us.ihmc.communication.ros2.tf2;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

public class ROS2FrameTest
{
   private static final double EPSILON = 1E-7;

   @Test
   public void testWorldFrame()
   {
      ROS2StaticFrame worldFrame = ROS2FrameTools.getWorldFrame();

      assertTrue(worldFrame.isWorldFrame());
      assertTrue(worldFrame.isRootFrame());
      assertTrue(worldFrame.isAStationaryFrame());
      assertTrue(worldFrame.isZupFrame());
      assertTrue(worldFrame.isFixedInParent());

      assertDoesNotThrow(worldFrame::update);

      assertNull(worldFrame.getParent());
      assertEquals(worldFrame, worldFrame.getRootFrame());
      assertEquals(ROS2FrameTools.WORLD_FRAME_ID, worldFrame.getFrameId());
      EuclidCoreTestTools.assertEquals(ReferenceFrame.getWorldFrame().getTransformToWorldFrame(), worldFrame.getTransformToWorldFrame(), EPSILON);
      EuclidCoreTestTools.assertEquals(ReferenceFrame.getWorldFrame().getTransformToRoot(), worldFrame.getTransformToRoot(), EPSILON);
   }

   @Test
   public void testConstructingStaticFrame()
   {
      ROS2Node node = new ROS2NodeBuilder().build("test_node");

      String id = "test_static_frame";
      ROS2Frame parentFrame = ROS2FrameTools.getWorldFrame();
      RigidBodyTransformReadOnly transformToParent = new RigidBodyTransform(new YawPitchRoll(0.1, 0.2, 0.3), new Vector3D(0.4, 0.5, 0.6));
      Instant timeAtConstruction = Instant.now();
      ROS2Frame staticFrame = new ROS2StaticFrame(node, id, parentFrame, transformToParent, timeAtConstruction);

      assertFalse(staticFrame.isWorldFrame());
      assertFalse(staticFrame.isRootFrame());
      assertFalse(staticFrame.isAStationaryFrame());
      assertFalse(staticFrame.isZupFrame());
      assertTrue(staticFrame.isFixedInParent());

      assertEquals(id, staticFrame.getFrameId());
      assertEquals(id, staticFrame.getName());
      assertEquals(timeAtConstruction, staticFrame.getLastUpdateTimestamp());
      assertEquals(parentFrame, staticFrame.getParent());
      EuclidCoreTestTools.assertEquals(transformToParent, staticFrame.getTransformToParent(), EPSILON);

      staticFrame.remove();
      node.destroy();
   }

   @Test
   public void testConstructingMutableFrame()
   {
      ROS2Node node = new ROS2NodeBuilder().build("test_node");

      String id = "test_mutable_frame";
      ROS2Frame parentFrame = ROS2FrameTools.getWorldFrame();
      ROS2MutableFrame mutableFrame = new ROS2MutableFrame(node, id, parentFrame);

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
      ROS2Node node = new ROS2NodeBuilder().build("test_node");

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
      ROS2Frame parentFrame = ROS2FrameTools.getWorldFrame();
      ROS2MutableFrame mutableFrame = new ROS2MutableFrame(node, id, parentFrame);

      // Run updates
      for (int i = 0; i < updatesToRun; ++i)
      {
         // Update the frame
         Instant updateTime = Instant.now();
         mutableFrame.updateTransform(updatedTransformToParent, updateTime);

         // Assert correct internal state
         assertEquals(updateTime, mutableFrame.getLastUpdateTimestamp());
         EuclidCoreTestTools.assertEquals(updatedTransformToParent, mutableFrame.getTransformToParent(), EPSILON);

         synchronized (correctMessagesReceived)
         {
            try
            {
               // Publish a message and wait for it to be received
               mutableFrame.publish();
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
}
