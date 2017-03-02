package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created by agrabertilton on 3/6/15.
 */
public class CompositeFootstepOverheadPath extends FootstepOverheadPath
{
   ReferenceFrame referenceFrame;
   double totalDistance = 0;
   FramePose2d endPose;
   ArrayList<FootstepOverheadPath> overheadPaths = new ArrayList<>();

   public CompositeFootstepOverheadPath(FootstepOverheadPath startingPath){
      this.referenceFrame = startingPath.getReferenceFrame();
      addPath(startingPath);
   }

   public CompositeFootstepOverheadPath(List<FootstepOverheadPath> startingPaths){
      for (FootstepOverheadPath path : startingPaths){
         if (referenceFrame == null){
            referenceFrame = path.getReferenceFrame();
         }else{
            this.checkReferenceFrameMatch(path);
         }
         addPath(path);
      }
   }

   public CompositeFootstepOverheadPath(FootstepOverheadPath[] startingPaths){
      for (FootstepOverheadPath path : startingPaths){
         if (referenceFrame == null){
            referenceFrame = path.getReferenceFrame();
         }else{
            this.checkReferenceFrameMatch(path);
         }
         addPath(path);
      }
   }

   public void addPath(FootstepOverheadPath pathToAdd){
      this.checkReferenceFrameMatch(pathToAdd);
      overheadPaths.add(pathToAdd);
      totalDistance += pathToAdd.getTotalDistance();
      endPose = pathToAdd.getPoseAtDistance(pathToAdd.getTotalDistance());
   }


   @Override
   public FramePose2d getPoseAtDistance(double distanceAlongPath)
   {
      double currentDistanceInPath = distanceAlongPath;
      if (distanceAlongPath > totalDistance){
         return endPose;
      }

      for (FootstepOverheadPath path : overheadPaths){
         if (currentDistanceInPath > path.getTotalDistance()){
            currentDistanceInPath -= path.getTotalDistance();
         }else{
            return path.getPoseAtDistance(currentDistanceInPath);
         }
      }

      //should not get here, means a numerical error somewhere
      return endPose;
   }

   @Override
   public double getTotalDistance()
   {
      return totalDistance;
   }

   @Override
   public FootstepOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      CompositeFootstepOverheadPath newPath = null;
      for (FootstepOverheadPath path : overheadPaths){
         if (newPath == null){
            newPath = new CompositeFootstepOverheadPath(path.changeFrameCopy(desiredFrame));
         }else{
            newPath.addPath(path.changeFrameCopy(desiredFrame));
         }
      }
      return newPath;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
