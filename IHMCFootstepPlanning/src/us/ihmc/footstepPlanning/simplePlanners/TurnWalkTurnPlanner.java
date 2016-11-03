package us.ihmc.footstepPlanning.simplePlanners;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class TurnWalkTurnPlanner implements FootstepPlanner
{

   @Override
   public void setInitialStanceFoot(FramePose2d stanceFootPose, RobotSide side)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setGoalPose(FramePose2d goalPose)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public List<FramePose2d> plan()
   {
      ArrayList<FramePose2d> ret = new ArrayList<>();
      ret.add(new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), 0.0));
      ret.add(new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.2), 0.0));
      ret.add(new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.1, 0.2), Math.PI/2.0));
      return ret;
   }

}
