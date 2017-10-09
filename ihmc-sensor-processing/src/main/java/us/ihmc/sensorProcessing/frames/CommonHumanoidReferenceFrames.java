package us.ihmc.sensorProcessing.frames;

import java.util.EnumMap;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;

public interface CommonHumanoidReferenceFrames extends ReferenceFrames
{
   public abstract MovingReferenceFrame getABodyAttachedZUpFrame();

   public abstract MovingReferenceFrame getMidFeetZUpFrame();

   public abstract MovingReferenceFrame getMidFeetUnderPelvisFrame();

   public abstract SideDependentList<MovingReferenceFrame> getAnkleZUpReferenceFrames();

   public abstract SideDependentList<MovingReferenceFrame> getFootReferenceFrames();

   public abstract SideDependentList<MovingReferenceFrame> getSoleFrames();

   public abstract MovingReferenceFrame getPelvisFrame();

   public abstract MovingReferenceFrame getAnkleZUpFrame(RobotSide robotSide);

   public abstract MovingReferenceFrame getFootFrame(RobotSide robotSide);

   public abstract MovingReferenceFrame getLegJointFrame(RobotSide robotSide, LegJointName legJointName);

   public abstract EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(RobotSide robotSide);

   public abstract ReferenceFrame getIMUFrame();

   public abstract MovingReferenceFrame getPelvisZUpFrame();

   public abstract MovingReferenceFrame getSoleFrame(RobotSide robotSide);

   public abstract MovingReferenceFrame getSoleZUpFrame(RobotSide robotSide);

   public abstract SideDependentList<MovingReferenceFrame> getSoleZUpFrames();

   public abstract MovingReferenceFrame getChestFrame();

   public abstract MovingReferenceFrame getMidFootZUpGroundFrame();
}
