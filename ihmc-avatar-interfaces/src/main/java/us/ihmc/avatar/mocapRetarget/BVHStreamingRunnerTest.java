package us.ihmc.avatar.mocapRetarget;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.mocapRetarget.bvh.BVHParser;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.JointInfo;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.avatar.mocapRetarget.bvh.MotionFrame;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class BVHStreamingRunnerTest
{
   static final String BVH_PATH = "/home/gwalrath/Downloads/walk_100.bvh";

   @Test
   void t00_fileExists()
   {
      File f = new File(BVH_PATH);
      assertTrue(f.exists(), "walk_100.bvh not found at " + f.getAbsolutePath());
      assertTrue(f.isFile() && f.canRead(), "walk_100.bvh not readable");
   }

   @Test
   void t01_parseHierarchyAndMotion() throws Exception
   {
      BVHParser parser = new BVHParser();
      SkeletonHierarchy skel = parser.parseHierarchy(new File(BVH_PATH));
      assertNotNull(skel, "SkeletonHierarchy is null");
      assertTrue(skel.joints.containsKey("Hips"), "No Hips joint");
      assertTrue(skel.joints.containsKey("Spine4"), "No Spine4 joint");
      assertTrue(skel.joints.containsKey("LeftFoot"), "No LeftFoot joint");
      assertTrue(skel.joints.containsKey("RightFoot"), "No RightFoot joint");

      List<MotionFrame> frames = parser.parseMotion(new File(BVH_PATH), skel);
      assertEquals(100, frames.size(), "Expected 100 frames");
      double ft = parser.getFrameTimeSeconds();
      assertTrue(ft > 0.0 && ft < 0.05, "Unexpected frame time (expected ~1/60 s)");
   }

   @Test
   void t02a_rawHipsPositions() throws Exception
   {
      BVHParser parser = new BVHParser();
      SkeletonHierarchy skel = parser.parseHierarchy(new File(BVH_PATH));
      List<MotionFrame> frames = parser.parseMotion(new File(BVH_PATH), skel);

      // Get index of Hips joint
      Map<String, JointInfo> joints = skel.joints;
      assertTrue(joints.containsKey("Hips"), "No Hips joint in skeleton");
      int hipsIndex = joints.get("Hips").channelStartIndex();

      // BVHParser should store channels per joint in a fixed order; root has 6 channels: Xpos,Ypos,Zpos,Zrot,Xrot,Yrot
      //int baseChannel = skel.getJointChannelStartIndex(hipsIndex);

      System.out.println("Hips frame 0 position (cm): "
                         + frames.get(0).channelData()[hipsIndex] + ", "
                         + frames.get(0).channelData()[hipsIndex + 1] + ", "
                         + frames.get(0).channelData()[hipsIndex + 2]);

      System.out.println("Hips frame 0 position (cm): "
                         + frames.get(50).channelData()[hipsIndex] + ", "
                         + frames.get(50).channelData()[hipsIndex + 1] + ", "
                         + frames.get(50).channelData()[hipsIndex + 2]);

      double dx = frames.get(50).channelData()[hipsIndex] - frames.get(0).channelData()[hipsIndex];
      double dy = frames.get(50).channelData()[hipsIndex + 1] - frames.get(0).channelData()[hipsIndex + 1];
      double dz = frames.get(50).channelData()[hipsIndex + 2] - frames.get(0).channelData()[hipsIndex + 2];

      assertFalse(Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6 && Math.abs(dz) < 1e-6,
                  "Root position identical between frames — no translation in BVH?");
   }

   @Test
   void t02b_rawHipsRotations() throws Exception
   {
      BVHParser parser = new BVHParser();
      SkeletonHierarchy skel = parser.parseHierarchy(new File(BVH_PATH));
      List<MotionFrame> frames = parser.parseMotion(new File(BVH_PATH), skel);

      // Get index of Hips joint
      Map<String, JointInfo> joints = skel.joints;
      assertTrue(joints.containsKey("Hips"), "No Hips joint in skeleton");
      int hipsIndex = joints.get("Hips").channelStartIndex();

      // BVHParser should store channels per joint in a fixed order; root has 6 channels: Xpos,Ypos,Zpos,Zrot,Xrot,Yrot
      //int baseChannel = skel.getJointChannelStartIndex(hipsIndex);

      double z0 = frames.get(0).channelData()[hipsIndex + 3];
      double x0 = frames.get(0).channelData()[hipsIndex + 4];
      double y0 = frames.get(0).channelData()[hipsIndex + 5];

      double z50 = frames.get(49).channelData()[hipsIndex + 3];
      double x50 = frames.get(49).channelData()[hipsIndex + 4];
      double y50 = frames.get(49).channelData()[hipsIndex + 5];


      boolean same = Math.abs(z50 - z0) < 1e-6 && Math.abs(x50 - x0) < 1e-6 && Math.abs(y50 - y0) < 1e-6;
      assertFalse(same, "Root rotations identical between frames — no rotation in BVH?");
   }


   // Helper: build quaternion from BVH Z-X-Y Euler angles (degrees), in that application order.
   private static Quaternion quatFromBvhZXY(double zDeg, double xDeg, double yDeg)
   {
      // Convert degrees to radians
      double z = Math.toRadians(zDeg);
      double x = Math.toRadians(xDeg);
      double y = Math.toRadians(yDeg);

      // Compose Rz * Rx * Ry
      Quaternion qz = new Quaternion();
      qz.appendYawRotation(z);

      Quaternion qx = new Quaternion();
      qx.appendRollRotation(x);

      Quaternion qy = new Quaternion();
      qy.appendPitchRotation(y);

      Quaternion q = new Quaternion(qz);
      q.multiply(qx);
      q.multiply(qy);
      return q;
   }

   @Test
   void t02d_tfRotationMatchesRootChannels() throws Exception
   {
      BVHParser parser = new BVHParser();
      SkeletonHierarchy skel = parser.parseHierarchy(new File(BVH_PATH));
      List<MotionFrame> frames = parser.parseMotion(new File(BVH_PATH), skel);

      Map<String, JointInfo> joints = skel.joints;
      assertTrue(joints.containsKey("Hips"), "No Hips");
      int hipsIndex = joints.get("Hips").channelStartIndex();
      //int base = skel.getJointChannelStartIndex(hipsIndex);

      // Check two frames
      int[] ks = {0, 49};

      CoordinateTransformer tf = new CoordinateTransformer(skel);

      for (int k : ks)
      {
         Map<String, RigidBodyTransform> g = tf.buildGlobalTransforms(frames.get(k));
         RigidBodyTransform H = g.get("Hips");
         assertNotNull(H, "No transform for Hips at frame " + k);

         // FK quaternion// Quaternion qFK = new Quaternion();
       RotationMatrixBasics rot = H.getRotation();

         // BVH root Euler Z, X, Y (deg)
         double z = frames.get(k).channelData()[hipsIndex + 3];
         double x = frames.get(k).channelData()[hipsIndex + 4];
         double y = frames.get(k).channelData()[hipsIndex + 5];
         Quaternion qBVH = quatFromBvhZXY(z, x, y);

//         // Compare quaternions (allow small numeric tolerance)
//         double dot = Math.abs(rot.getS() * qBVH.getS() + rot.getX() * qBVH.getX()
//                               + qFK.getY() * qBVH.getY() + qFK.getZ() * qBVH.getZ());
//         assertTrue(dot > 0.999, "FK rotation != BVH Z-X-Y rotation at frame " + k + " (dot=" + dot + ")");
      }
   }

   @Test
   void t02c_tfMatchesRootChannels() throws Exception
   {
      BVHParser parser = new BVHParser();
      SkeletonHierarchy skel = parser.parseHierarchy(new File(BVH_PATH));
      List<MotionFrame> frames = parser.parseMotion(new File(BVH_PATH), skel);

      Map<String, JointInfo> joints = skel.joints;
      assertTrue(joints.containsKey("Hips"), "No Hips");
      int hipsIndex = joints.get("Hips").channelStartIndex();
      //int base = skel.getJointChannelStartIndex(hipsIndex);

      // pick two frames
      int[] ks = {0, 49};

      CoordinateTransformer tf = new CoordinateTransformer(skel);

      for (int k : ks)
      {
         Map<String, RigidBodyTransform> g = tf.buildGlobalTransforms(frames.get(k));
         RigidBodyTransform H = g.get("Hips");
         assertNotNull(H, "No transform for Hips at frame " + k);

         double cx = frames.get(k).channelData()[hipsIndex];
         double cy = frames.get(k).channelData()[hipsIndex + 1];
         double cz = frames.get(k).channelData()[hipsIndex + 2];

         // Extract translation from transform
         double[] T = new double[16];
         H.get(T);
         double tx = T[12], ty = T[13], tz = T[14];

         // Compare within small tolerance (units should match BVH — cm)
         assertEquals(cx, tx, 1e-4, "FK tx != BVH Xpos at frame " + k);
         assertEquals(cy, ty, 1e-4, "FK ty != BVH Ypos at frame " + k);
         assertEquals(cz, tz, 1e-4, "FK tz != BVH Zpos at frame " + k);
      }
   }

//   @Test
//   void t02_buildGlobalTransforms() throws Exception
//   {
//      BVHParser parser = new BVHParser();
//      SkeletonHierarchy skel = parser.parseHierarchy(new File(BVH_PATH));
//      List<MotionFrame> frames = parser.parseMotion(new File(BVH_PATH), skel);
//
//      CoordinateTransformer tf = new CoordinateTransformer(skel);
//      Map<String, RigidBodyTransform> g0 = tf.buildGlobalTransforms(frames.get(0));
//      Map<String, RigidBodyTransform> g50 = tf.buildGlobalTransforms(frames.get(50));
//
//      assertNotNull(g0.get("Hips"), "No transform for Hips frame 0");
//      assertNotNull(g50.get("Hips"), "No transform for Hips frame 50");
//
//      RigidBodyTransform diff = new RigidBodyTransform(g0.get("Hips"));
//      diff.invert();
//      diff.multiply(g50.get("Hips"));
//
//      assertFalse(diff.epsilonEquals(new RigidBodyTransform(), 1e-6),
//                  "Hips transform is identical between frames — no motion?");
//   }

}
