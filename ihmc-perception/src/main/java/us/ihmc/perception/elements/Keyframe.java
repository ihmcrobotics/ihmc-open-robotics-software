package us.ihmc.perception.elements;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;

public class Keyframe
{
   int id = -1;
   private ArrayList<Integer> landmarkIDs;
   private FramePose3D pose;

   public Keyframe(int id)
   {
      this.id = id;
      landmarkIDs = new ArrayList<>();
      pose = new FramePose3D();
   }

   public int getId()
   {
      return id;
   }

   public FramePose3D getPose()
   {
      return pose;
   }

   public void setPose(FramePose3D pose)
   {
      this.pose = pose;
   }

   public int getLandmarkCount()
   {
      return landmarkIDs.size();
   }

   public void addLandmark(int id)
   {
      landmarkIDs.add(id);
   }
}
