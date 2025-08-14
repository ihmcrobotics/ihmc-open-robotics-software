package us.ihmc.avatar.mocapRetarget;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.avatar.mocapRetarget.bvh.BVHParser;
import us.ihmc.avatar.mocapRetarget.bvh.MotionFrame;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.io.File;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

import us.ihmc.ros2.ROS2Node;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2QosProfile;

/**
 * End-to-end runner:
 *  - Parses a BVH file
 *  - Builds per-frame BVH globals
 *  - Calibrates BVH -> WORLD (pelvis)
 *  - Retargets to Kinematics Streaming Toolbox message
 *  - Sends each frame via provided 'sender'
 *
 * Usage sketch (see main() below):
 *   BVHStreamingRunner runner = BVHStreamingRunner.create(fullModel, new File("path/to/walking.bvh"));
 *   runner.run(sender); // 'sender' is a ROS2 publisher wrapper; see the two variants in main()
 */
public class BVHStreamingRunner
{
   private final FullHumanoidRobotModel model;
   private final SkeletonHierarchy skeleton;
   private final List<MotionFrame> frames;
   private final double frameTimeSeconds;
   private final CoordinateTransformer transformer;
   private final JointMap jointMap;

   private long sequenceId = 1L;

   private BVHStreamingRunner(FullHumanoidRobotModel model,
                              SkeletonHierarchy skeleton,
                              List<MotionFrame> frames,
                              double frameTimeSeconds,
                              CoordinateTransformer transformer,
                              JointMap jointMap)
   {
      this.model = model;
      this.skeleton = skeleton;
      this.frames = frames;
      this.frameTimeSeconds = frameTimeSeconds;
      this.transformer = transformer;
      this.jointMap = jointMap;
   }

   /** Factory: parse BVH, set up transformer + joint map, add end-effector mappings, and calibrate pelvis. */
   public static BVHStreamingRunner create(FullHumanoidRobotModel model, File bvhFile) throws Exception
   {
      Objects.requireNonNull(model, "FullHumanoidRobotModel required");
      Objects.requireNonNull(bvhFile, "BVH file required");

      // 1) Parse BVH
      BVHParser parser = new BVHParser();
      SkeletonHierarchy skeleton = parser.parseHierarchy(bvhFile);
      List<MotionFrame> frames = parser.parseMotion(bvhFile, skeleton);
      if (frames.isEmpty())
         throw new IllegalArgumentException("BVH contains zero frames: " + bvhFile);

      // 2) Frame time (seconds). If your parser exposes it, prefer that. Else default to 60 Hz.
      double frameTimeSeconds = 1.0 / 60.0;
      try
      {
         frameTimeSeconds = parser.getFrameTimeSeconds(); // add this getter in your BVHParser if not present
      }
      catch (Throwable ignore) { /* fallback OK */ }

      // 3) FK transformer
      CoordinateTransformer transformer = new CoordinateTransformer(skeleton);

      // 4) JointMap + exact end-effector mappings for your BVH
      JointMap jointMap = new JointMap(skeleton, model);
      addEndEffectorMappingsForWalkingBVH(jointMap, model); // <<<<<<<<<<<<<< EXACT MAPPINGS

      // Optional: if your actor height differs from robot height, set a scale:
      // jointMap.setPositionScale(robotHeight / bvhActorHeight);

      // 5) Calibrate BVH->WORLD once using pelvis at frame 0 (yaw-only to avoid BVH tilt)
      Map<String, RigidBodyTransform> globals0 = transformer.buildGlobalTransforms(frames.get(0));
      RigidBodyTransform pelvisWorld = getBodyWorldTransform(model.getPelvis());
      jointMap.calibrateBvhToWorldFromPelvis("Hips", pelvisWorld, globals0, /*alignYawOnly=*/true);

      return new BVHStreamingRunner(model, skeleton, frames, frameTimeSeconds, transformer, jointMap);
   }

   /** Stream all frames at the BVH frame rate to the provided sender (ROS 2 publisher, etc). */
   public void run(Consumer<KinematicsStreamingToolboxInputMessage> sender) throws InterruptedException
   {
      long nanosPerFrame = (long) (frameTimeSeconds * 1e9);

      for (int i = 0; i < frames.size(); i++)
      {
         long t0 = System.nanoTime();

         MotionFrame frame = frames.get(i);
         Map<String, RigidBodyTransform> bvhGlobals = transformer.buildGlobalTransforms(frame);

         KinematicsStreamingToolboxInputMessage msg =
               jointMap.toStreamingMessage(bvhGlobals, sequenceId++, /*timestamp*/ t0, /*streamToController*/ true);

         sender.accept(msg);

         // Sleep best-effort to match BVH rate
         long elapsed = System.nanoTime() - t0;
         long remaining = nanosPerFrame - elapsed;
         if (remaining > 0)
            Thread.sleep(remaining / 1_000_000L, (int) (remaining % 1_000_000L));
      }
   }

   // ------------------------
   // EXACT end-effector mapping for your walking.bvh
   // ------------------------
   private static void addEndEffectorMappingsForWalkingBVH(JointMap jointMap, FullHumanoidRobotModel model)
   {
      // Pelvis & a chest proxy (Spine4 is the highest spine joint present)
      safeAddEndEffector(jointMap, "Hips",   model.getPelvis());
      safeAddEndEffector(jointMap, "Spine4", model.getChest()); // if chest control is desired

      // Hands & feet (BVH names from your file)
      safeAddEndEffector(jointMap, "LeftHand",  model.getHand(RobotSide.LEFT));
      safeAddEndEffector(jointMap, "RightHand", model.getHand(RobotSide.RIGHT));
      safeAddEndEffector(jointMap, "LeftFoot",  model.getFoot(RobotSide.LEFT));
      safeAddEndEffector(jointMap, "RightFoot", model.getFoot(RobotSide.RIGHT));

      // Optional: Head control (if your robot exposes a head rigid body)
      // safeAddEndEffector(jointMap, "Head", model.getHead());

      // Optional: Forearms (if you want more constraints)
      // safeAddEndEffector(jointMap, "LeftForeArm",  model.getForearm(RobotSide.LEFT));
      // safeAddEndEffector(jointMap, "RightForeArm", model.getForearm(RobotSide.RIGHT));
   }

   private static void safeAddEndEffector(JointMap jointMap, String bvhName, RigidBodyBasics body)
   {
      if (body != null)
         jointMap.addEndEffectorMapping(bvhName, body);
   }

   /** Helper: body’s WORLD transform (used for calibration target). */
   private static RigidBodyTransform getBodyWorldTransform(RigidBodyBasics body)
   {
      RigidBodyTransform out = new RigidBodyTransform();
      MovingReferenceFrame frame = body.getBodyFixedFrame();
      frame.getTransformToDesiredFrame(out, ReferenceFrame.getWorldFrame());
      return out;
   }


   public static void main(String[] args) throws Exception
   {
     // FullHumanoidRobotModel model = ;

      File bvhFile = new File("/home/gwalrath/Downloads/walking.bvh");
      //BVHStreamingRunner runner = BVHStreamingRunner.create(model, bvhFile);

     // ROS2Node ros2 = ROS2Tools.createROS2Node("bvh_streaming_runner");

      // Publisher
      //ROS2Publisher<KinematicsStreamingToolboxInputMessage> pub =
      //      ros2.createPublisher(KinematicsStreamingToolboxInputMessage.class,
                               //  "/ihmc/kinematics_streaming_toolbox/input",
                               //  ROS2QosProfile.DEFAULT());

      //runner.run(pub::publish);

   }

}

