package us.ihmc.avatar.mocapRetarget.bvh;

import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.JointInfo;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.rmi.server.Skeleton;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Stack;

public class BVHParser
{
   /**
    * Parses the HIERARCHY section and returns a skeleton description
    */
   public SkeletonHierarchy parseHierarchy(File bvhFile) throws IOException
   {
      SkeletonHierarchy skeleton = new SkeletonHierarchy();
      try (BufferedReader reader = new BufferedReader(new FileReader(bvhFile))) {
         String line;
         while((line=reader.readLine()) != null && !(line.trim()).equals("MOTION")) {
            if (line.trim().equals("HIERARCHY")) {
               if ((line=reader.readLine().trim()).startsWith("ROOT")) {
                  Stack<String> parents = new Stack<>();
                  parents.push(null);
                  parseJoints(reader, skeleton, line.trim(), parents);
                  break;
               }
            }
         }
      }
      return skeleton;
   }

      private void parseJoints(BufferedReader reader, SkeletonHierarchy skeleton, String line, Stack<String> parents) throws IOException
      {
         String name = line.split("\\s+")[1];
         List<String> channels = new ArrayList<>();
         String parentName = parents.peek();


         reader.readLine();
         Vector3D offset = getOffset(reader.readLine().trim());


         if (!(name.equals("End Site"))) {
            channels = getChannels(reader.readLine().trim());
         }

         skeleton.addJoint(new JointInfo(name, parentName, offset, channels));
         parents.push(name);
         line = reader.readLine().trim();
         while (line.startsWith("JOINT") || line.startsWith("End Site")) {
            parseJoints(reader, skeleton, line, parents);
            line = reader.readLine().trim();
         }

         parents.pop();
         return;
      }

      private List<String> getChannels(String line) {
         String[] lineSplit = line.split("\\s+");
         return new ArrayList<String>(Arrays.asList(lineSplit).subList(2, lineSplit.length));
      }

      private Vector3D getOffset(String line) {
         String[] lineSplit = line.split("\\s+");
         double[] vectorArray = new double[3];
         for (int i = 1; lineSplit.length > i; i++) {
            vectorArray[i-1] = Double.parseDouble(lineSplit[i]);
         }
         return new Vector3D(vectorArray);
      }


   /*
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

   public static void main(String[] args) {
      try {
         File bvhFile = new File(args[0]);
         BVHParser parser = new BVHParser();
         SkeletonHierarchy hierarchy = parser.parseHierarchy(bvhFile);

         System.out.println("Hierarchy:");
         for (String jointName: hierarchy.getJointNames()) {
            JointInfo joint = hierarchy.getJoint(jointName);
            System.out.println(joint.parentName());
            System.out.println(jointName + ", Offset: " + joint.offset() + ", Channels: " + joint.channels()  );
         }

      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }
}

