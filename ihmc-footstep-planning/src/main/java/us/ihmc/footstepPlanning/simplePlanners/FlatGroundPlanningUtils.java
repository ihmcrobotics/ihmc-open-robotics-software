package us.ihmc.footstepPlanning.simplePlanners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

public class FlatGroundPlanningUtils
{
   public static FramePose3D poseFormPose2d(FramePose2D pose2d)
   {
      FramePose3D pose = new FramePose3D(pose2d.getReferenceFrame());
      pose.setOrientationYawPitchRoll(pose2d.getYaw(), 0.0, 0.0);
      pose.setX(pose2d.getX());
      pose.setY(pose2d.getY());
      return pose;
   }

   public static FramePose3D poseFormPose2d(FramePose2D pose2d, double z)
   {
      FramePose3D pose = poseFormPose2d(pose2d);
      pose.setZ(z);
      return pose;
   }

   public static FramePose2D pose2dFormPose(FramePose3D pose)
   {
      FramePose2D pose2d = new FramePose2D(pose.getReferenceFrame());
      pose2d.setYaw(pose.getYaw());
      pose2d.setX(pose.getX());
      pose2d.setY(pose.getY());
      return pose2d;
   }

   public static List<FramePose3D> poseListFromPoseList2d(List<FramePose2D> pose2dList)
   {
      ArrayList<FramePose3D> poseList = new ArrayList<>();
      for (FramePose2D pose2d : pose2dList)
         poseList.add(poseFormPose2d(pose2d));
      return poseList;
   }

   public static List<FramePose3D> poseListFromPoseList2d(List<FramePose2D> pose2dList, double z)
   {
      ArrayList<FramePose3D> poseList = new ArrayList<>();
      for (FramePose2D pose2d : pose2dList)
         poseList.add(poseFormPose2d(pose2d, z));
      return poseList;
   }

   public static List<FramePose2D> pose2dListFromPoseList(List<FramePose3D> poseList)
   {
      ArrayList<FramePose2D> pose2dList = new ArrayList<>();
      for (FramePose3D pose : poseList)
         pose2dList.add(pose2dFormPose(pose));
      return pose2dList;
   }
}
