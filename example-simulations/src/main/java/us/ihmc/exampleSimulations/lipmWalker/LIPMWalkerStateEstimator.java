package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class LIPMWalkerStateEstimator
{
   private final PoseReferenceFrame comReferenceFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
   private final SideDependentList<PoseReferenceFrame> zUpFootFrames = new SideDependentList<>();

   private final LIPMWalkerRobot robot;

   public LIPMWalkerStateEstimator(LIPMWalkerRobot robot)
   {
      this.robot = robot;

      for (RobotSide robotSide : RobotSide.values)
      {
         zUpFootFrames.put(robotSide, new PoseReferenceFrame(robotSide.getLowerCaseName() + "ZUpFootFrame", ReferenceFrame.getWorldFrame()));
      }
   }

   public void update()
   {
      comReferenceFrame.setPositionAndUpdate(robot.getCenterOfMassPosition());

      for (RobotSide robotSide : RobotSide.values)
         zUpFootFrames.get(robotSide).setPositionAndUpdate(robot.getFootPosition(robotSide));
   }

   public ReferenceFrame getCoMReferenceFrame()
   {
      return comReferenceFrame;
   }

   public ReferenceFrame getFootFrame(RobotSide robotSide)
   {
      return zUpFootFrames.get(robotSide);
   }
}
