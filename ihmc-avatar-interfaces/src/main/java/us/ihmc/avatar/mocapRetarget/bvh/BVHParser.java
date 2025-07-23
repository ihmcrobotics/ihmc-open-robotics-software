package us.ihmc.avatar.mocapRetarget.bvh;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseKinematicsSolver;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class BVHParser
{
   /**
    * Parses the HIERARCHY section and returns a skeleton description
    * (joint names, parent relationships, offsets, channel orders).
    */
   public SkeletonHierarchy parseHierarchy(File bvhFile) throws IOException
   {
      SkeletonHierarchy hierarchy = new SkeletonHierarchy();
      // 1) open a BufferedReader on bvhFile
      // 2) read lines until you see "HIERARCHY"
      // 3) recursively parse each JOINT / End Site block:
      //      - read OFFSET x y z
      //      - read CHANNELS count {channel names}
      //      - for each child JOINT, call yourself
      // 4) stop when you hit "MOTION"
      return hierarchy;
   }

   /**
    * Parses the MOTION section after hierarchy, returning a list of frames:
    *   - frameCount
    *   - frameTime (dt)
    *   - for each frame, the flat float[] of channel values
    */
 /*  public List<MotionFrame> parseMotion(File bvhFile, SkeletonHierarchy hierarchy) throws IOException
   {
      // 1) skip ahead to "MOTION"
      // 2) read "Frames: N"  → int frameCount
      // 3) read "Frame Time: dt" → double frameTime
      // 4) for i=0..N-1:
      //      - read a line, split on whitespace → float[channelCount]
      //      - new MotionFrame(i * dt, array)
      // 5) return list of MotionFrame

   }
   */
}
