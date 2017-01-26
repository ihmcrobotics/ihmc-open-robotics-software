package us.ihmc.footstepPlanning.simplePlanners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;

public class FlatGroundPlanningUtils
{
   public static FramePose poseFormPose2d(FramePose2d pose2d)
   {
      FramePose pose = new FramePose(pose2d.getReferenceFrame());
      pose.setYawPitchRoll(pose2d.getYaw(), 0.0, 0.0);
      pose.setX(pose2d.getX());
      pose.setY(pose2d.getY());
      return pose;
   }

   public static FramePose poseFormPose2d(FramePose2d pose2d, double z)
   {
      FramePose pose = poseFormPose2d(pose2d);
      pose.setZ(z);
      return pose;
   }

   public static FramePose2d pose2dFormPose(FramePose pose)
   {
      FramePose2d pose2d = new FramePose2d(pose.getReferenceFrame());
      pose2d.setYaw(pose.getYaw());
      pose2d.setX(pose.getX());
      pose2d.setY(pose.getY());
      return pose2d;
   }

   public static List<FramePose> poseListFromPoseList2d(List<FramePose2d> pose2dList)
   {
      ArrayList<FramePose> poseList = new ArrayList<>();
      for (FramePose2d pose2d : pose2dList)
         poseList.add(poseFormPose2d(pose2d));
      return poseList;
   }

   public static List<FramePose> poseListFromPoseList2d(List<FramePose2d> pose2dList, double z)
   {
      ArrayList<FramePose> poseList = new ArrayList<>();
      for (FramePose2d pose2d : pose2dList)
         poseList.add(poseFormPose2d(pose2d, z));
      return poseList;
   }

   public static List<FramePose2d> pose2dListFromPoseList(List<FramePose> poseList)
   {
      ArrayList<FramePose2d> pose2dList = new ArrayList<>();
      for (FramePose pose : poseList)
         pose2dList.add(pose2dFormPose(pose));
      return pose2dList;
   }
}
