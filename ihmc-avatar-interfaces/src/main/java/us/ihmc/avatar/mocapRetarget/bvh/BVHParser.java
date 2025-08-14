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
   private double frameTimeSeconds;
   /**
    * Parses the HIERARCHY section and returns a skeleton description
    */
   public SkeletonHierarchy parseHierarchy(File bvhFile) throws IOException
   {
      SkeletonHierarchy skeleton = new SkeletonHierarchy();
      int indexTracker = 0;
      try (BufferedReader reader = new BufferedReader(new FileReader(bvhFile)))
      {
         String line;
         while ((line = reader.readLine()) != null && !(line.trim()).equals("MOTION"))
         {
            if (line.trim().equals("HIERARCHY"))
            {
               if ((line = reader.readLine().trim()).startsWith("ROOT"))
               {
                  Stack<String> parents = new Stack<>();
                  parents.push(null);
                  indexTracker = parseJoints(reader, skeleton, line.trim(), parents, indexTracker);
                  break;
               }
            }
         }
      }
      return skeleton;
   }

   private int parseJoints(BufferedReader reader, SkeletonHierarchy skeleton, String line, Stack<String> parents, int startIndex) throws IOException
   {
      String[] splitLine = line.split("\\s+");
      String name;
      List<String> channels = new ArrayList<>();
      int channelCount = 0;
      int indexTracker = startIndex;
      String parentName = parents.peek();

      if (line.startsWith("End Site"))
      {
         name = splitLine[0] + " " + splitLine[1] + "-" + parentName;
      }
      else
      {
         name = splitLine[1];
      }

      reader.readLine();
      Vector3D offset = getOffset(reader.readLine().trim());

      if (!(name.startsWith("End Site")))
      {
         channels = getChannels(reader.readLine().trim());
         channelCount = channels.size();
         indexTracker += channelCount;
      }

      skeleton.addJoint(new JointInfo(name, parentName, offset, channels, channelCount, startIndex));
      parents.push(name);
      line = reader.readLine().trim();
      while (line.startsWith("JOINT") || line.startsWith("End Site"))
      {
         indexTracker = parseJoints(reader, skeleton, line, parents, indexTracker);
         line = reader.readLine().trim();
      }

      parents.pop();
      return indexTracker;
   }

   private List<String> getChannels(String line)
   {
      String[] lineSplit = line.split("\\s+");
      return new ArrayList<String>(Arrays.asList(lineSplit).subList(2, lineSplit.length));
   }

   private Vector3D getOffset(String line)
   {
      String[] lineSplit = line.split("\\s+");
      double[] vectorArray = new double[3];
      for (int i = 1; lineSplit.length > i; i++)
      {
         vectorArray[i - 1] = Double.parseDouble(lineSplit[i]);
      }
      return new Vector3D(vectorArray);
   }

   public List<MotionFrame> parseMotion(File bvhFile, SkeletonHierarchy hierarchy) throws IOException
   {
      int frameCount = 0;
      int channelCount = hierarchy.getTotalChannels();
      List<MotionFrame> frames = new ArrayList<MotionFrame>();

      try (BufferedReader reader = new BufferedReader(new FileReader(bvhFile)))
      {
         String line;
         while ((line = reader.readLine()) != null)
         {
            if (line.trim().equals("MOTION"))
            {
               frameCount = Integer.parseInt(reader.readLine().trim().split("\\s+")[1]);
               frameTimeSeconds = Double.parseDouble(reader.readLine().trim().split("\\s+")[2]);
               for (int i = 0; i < frameCount; i++) {
                  int index = 0;
                  double[] channelData = new double[channelCount];
                  String[] stringData = reader.readLine().trim().split("\\s+");
                  for (String num : stringData) {
                     channelData[index++] = Double.parseDouble(num);
                  }
                  frames.add(new MotionFrame(i+1, frameTimeSeconds*i, channelData));
               }
            }
         }
      }
      return frames;
   }

   // Getter method
   public double getFrameTimeSeconds()
   {
      return frameTimeSeconds;
   }

   public static void main(String[] args)
   {
      try
      {
         File bvhFile = new File(args[0]);
         BVHParser parser = new BVHParser();
         SkeletonHierarchy hierarchy = parser.parseHierarchy(bvhFile);
         List<MotionFrame> frames = parser.parseMotion(bvhFile, hierarchy);

         System.out.println("Hierarchy:");
         for (String jointName : hierarchy.getJointNames())
         {
            JointInfo joint = hierarchy.getJoint(jointName);
            System.out.println("\n" + joint.parentName());
            System.out.println(jointName + "\nOffset: " + joint.offset() + "\nChannels: " + joint.channels() + "\nChannel Count: " + joint.channelCount()
            + "\nChannel Start Index: " + joint.channelStartIndex());
         }
         System.out.println(hierarchy.getTotalChannels());
         System.out.println("\nMOTION:\n");
         System.out.println(frames.get(43241));
         System.out.println(frames.get(43241).channelData()[0]);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}

