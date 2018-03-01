package us.ihmc.sensorProcessing.frames;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;

public interface CommonHumanoidReferenceFrames extends CommonLeggedReferenceFrames<RobotSide>
{
   SideDependentList<MovingReferenceFrame> getAnkleZUpReferenceFrames();

   SideDependentList<MovingReferenceFrame> getFootReferenceFrames();

   SideDependentList<MovingReferenceFrame> getSoleFrames();

   SideDependentList<MovingReferenceFrame> getSoleZUpFrames();
}
