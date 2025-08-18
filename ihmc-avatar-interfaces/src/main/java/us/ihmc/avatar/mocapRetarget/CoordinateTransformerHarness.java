package us.ihmc.avatar.mocapRetarget;

import us.ihmc.avatar.mocapRetarget.bvh.BVHParser;
import us.ihmc.avatar.mocapRetarget.bvh.MotionFrame;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;

   /**
    * CoordinateTransformer Testing Harness
    * Writes a tiny BVH to a temp file, parses it, and prints global transforms for frame 0.
    */
   public class CoordinateTransformerHarness
   {
      public static void main(String[] args) throws Exception
      {
         File tempBVH = writeMiniBVH();

         // 2) Parse hierarchy + motion
         BVHParser parser = new BVHParser();
         SkeletonHierarchy skeleton = parser.parseHierarchy(tempBVH);
         List<MotionFrame> frames = parser.parseMotion(tempBVH, skeleton);

         System.out.println("Parsed joints: " + skeleton.joints.keySet());
         System.out.println("Frames: " + frames.size());

         // 3) Build globals for frame 0
         CoordinateTransformer transformer = new CoordinateTransformer(skeleton);
         MotionFrame f0 = frames.get(0);
         Map<String, RigidBodyTransform> globals0 = transformer.buildGlobalTransforms(f0);

         // 4) Print a couple of joints (root + child)
         System.out.println("\n=== Frame 0 Globals ===");
         globals0.forEach((name, T) -> {
            System.out.println("Joint: " + name);
            printTransform(T);
         });

         if (frames.size() > 1)
         {
            Map<String, RigidBodyTransform> globals1 = transformer.buildGlobalTransforms(frames.get(1));
            System.out.println("\n=== Frame 1 Globals ===");
            globals1.forEach((name, T) -> {
               System.out.println("Joint: " + name);
               printTransform(T);
            });
         }

         System.out.println("\nHarness complete: " + tempBVH.getAbsolutePath());
      }

      private static File writeMiniBVH() throws IOException
      {
         // ZXY rotation order on purpose to exercise channel order handling
         String bvh =
               "HIERARCHY\n" +
               "ROOT Hips\n" +
               "{\n" +
               "  OFFSET 0.0 0.0 0.0\n" +
               "  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n" +
               "  JOINT Knee\n" +
               "  {\n" +
               "    OFFSET 0.0 -0.4 0.0\n" +
               "    CHANNELS 3 Zrotation Xrotation Yrotation\n" +
               "    End Site\n" +
               "    {\n" +
               "      OFFSET 0.0 -0.4 0.0\n" +
               "    }\n" +
               "  }\n" +
               "}\n" +
               "MOTION\n" +
               "Frames: 3\n" +
               "Frame Time: 0.0166667\n" +
               // Frame 0: pelvis at origin, all angles 0
               "0 0 0   0 0 0    0 0 0\n" +
               // Frame 1: pelvis translate slightly + some Z,X,Y rotations
               "0.01 0.0 0.0   10 5 3    -5 2 0\n" +
               // Frame 2: more motion
               "0.02 0.0 0.0   20 10 6   -10 4 0\n";

         File tmp = File.createTempFile("mini", ".bvh");
         try (FileWriter fw = new FileWriter(tmp))
         {
            fw.write(bvh);
         }
         return tmp;
      }

      private static void printTransform(RigidBodyTransform T)
      {
         // Print translation
         double x = T.getTranslation().getX();
         double y = T.getTranslation().getY();
         double z = T.getTranslation().getZ();


         System.out.println("  R:\n" + T.getRotation());
      }
   }
