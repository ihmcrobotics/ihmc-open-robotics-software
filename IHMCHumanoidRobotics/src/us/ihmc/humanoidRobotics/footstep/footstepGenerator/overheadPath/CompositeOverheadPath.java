package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CompositeOverheadPath extends OverheadPath
{
   private static final double EPSILON = 1e-4;
   protected final ArrayList<OverheadPath> paths = new ArrayList<OverheadPath>();
   protected final ReferenceFrame referenceFrame;
   private OverheadPath currentPath;

   public CompositeOverheadPath(List<? extends OverheadPath> paths)
   {
      this.referenceFrame = paths.get(0).getReferenceFrame();

      for (OverheadPath overheadPath : paths)
      {
         addPath(overheadPath);
      }
   }

   public CompositeOverheadPath(OverheadPath path)
   {
      this.referenceFrame = path.getReferenceFrame();
      paths.add(path);
   }

   public void addPath(OverheadPath newPath)
   {
      checkReferenceFrameMatch(newPath);
      checkPoseContinuity(newPath);
      paths.add(newPath);
   }

   private void checkPoseContinuity(OverheadPath newPath)
   {
      if (paths.size() > 0)
      {
         FramePose2d lastPose = getPoseAtS(1.0);
         FramePose2d nextPose = newPath.getPoseAtS(0.0);
         if (!lastPose.epsilonEquals(nextPose, EPSILON))
            throw new PoseDiscontinuityException("lastPose " + lastPose + " does not match next pose " + nextPose + " in CompositeOverheadPath");
      }
   }

   @Override
   public FramePose2d getPoseAtS(double pathVariableS)
   {
      double subpathVariablS = getSForSubpath(pathVariableS);

      return currentPath.getPoseAtS(subpathVariablS);
   }

   @Override
   public FramePose2d getExtrapolatedPoseAtS(double pathVariableS)
   {
      if (pathVariableS > 1.0)
         return getPoseAtS((pathVariableS - 1) * paths.size(), 0);
      else if (pathVariableS < 0.0)
         return getPoseAtS(pathVariableS * paths.size(), paths.size() - 1);
      else
         return getPoseAtS(pathVariableS);
   }

   //getPoseAtS allows "negative" pathVariableS values (or values greater than 1) without crossing onto a different segment.
   public FramePose2d getPoseAtS(double subpathVariablS, int pathIndex)
   {
      currentPath = paths.get(pathIndex);

      return currentPath.getExtrapolatedPoseAtS(subpathVariablS);
   }

   private double getSForSubpath(double pathVariableS)
   {
      pathVariableS = Math.max(0.0, pathVariableS);
      double compositeS = pathVariableS * paths.size();
      int index = (int) Math.floor(compositeS);

      if (index == paths.size())
      {
         index = paths.size() - 1;
      }

      double remainderS = compositeS - index;

      currentPath = paths.get(index);

      return remainderS;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public CompositeOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      CompositeOverheadPath ret = null;
      for (OverheadPath path : this.paths)
      {
         if (ret == null)
            ret = new CompositeOverheadPath(path.changeFrameCopy(desiredFrame));
         else
            ret.addPath(path.changeFrameCopy(desiredFrame));
      }

      return ret;
   }
}
