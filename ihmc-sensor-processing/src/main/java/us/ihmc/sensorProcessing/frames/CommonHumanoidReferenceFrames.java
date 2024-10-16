package us.ihmc.sensorProcessing.frames;

import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;

public interface CommonHumanoidReferenceFrames extends CommonLeggedReferenceFrames<RobotSide>
{
   SideDependentList<MovingReferenceFrame> getAnkleZUpReferenceFrames();

   SideDependentList<MovingReferenceFrame> getFootReferenceFrames();

   SideDependentList<MovingReferenceFrame> getSoleFrames();

   SideDependentList<MovingReferenceFrame> getSoleZUpFrames();

   MovingReferenceFrame getPelvisFrame();

   MovingReferenceFrame getPelvisZUpFrame();

   MovingReferenceFrame getChestFrame();

   MovingReferenceFrame getMidFeetZUpFrame();

   MovingReferenceFrame getMidFeetUnderPelvisFrame();

   MovingReferenceFrame getABodyAttachedZUpFrame();

   MovingReferenceFrame getMidFootZUpGroundFrame();
}
