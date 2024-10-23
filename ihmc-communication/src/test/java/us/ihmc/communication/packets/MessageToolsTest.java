package us.ihmc.communication.packets;

import ihmc_common_msgs.msg.dds.PoseListMessage;
import ihmc_common_msgs.msg.dds.UUIDMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.UUID;

import static org.junit.jupiter.api.Assertions.*;

public class MessageToolsTest
{
   @Test
   public void testPoseListMessage()
   {
      Random random = new Random();
      Pose3D pose0 = new Pose3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextQuaternion(random));
      Pose3D pose1 = new Pose3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextQuaternion(random));

      ArrayList<Pose3D> poses = new ArrayList<>();
      poses.add(pose0);
      poses.add(pose1);

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      List<Pose3D> posesAfter = MessageTools.unpackPoseListMessage(poseListMessage);

      EuclidCoreTestTools.assertEquals(pose0, posesAfter.get(0), 1e-5);
      EuclidCoreTestTools.assertEquals(pose1, posesAfter.get(1), 1e-5);
   }

   @Test
   public void testSerializeDeserialize()
   {
      UUID randomUUID = UUID.randomUUID();
      UUIDMessage uuidMessage = new UUIDMessage();

      MessageTools.toMessage(randomUUID, uuidMessage);

      ByteBuffer serializedMessage = MessageTools.serialize(uuidMessage);

      UUIDMessage deserializedMessage = new UUIDMessage();
      MessageTools.deserialize(serializedMessage, deserializedMessage);

      UUID fromMessage = MessageTools.toUUID(deserializedMessage);

      assertEquals(randomUUID, fromMessage);
   }
}
