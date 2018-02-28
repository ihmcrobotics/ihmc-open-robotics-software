package us.ihmc.sensorProcessing.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;

import java.util.EnumMap;

public interface CommonLeggedReferenceFrames<E extends Enum<E> & RobotSegment<E>> extends ReferenceFrames
{
   MovingReferenceFrame getABodyAttachedZUpFrame();

   MovingReferenceFrame getMidFeetZUpFrame();

   MovingReferenceFrame getMidFeetUnderPelvisFrame();

   SegmentDependentList<E, MovingReferenceFrame> getAnkleZUpReferenceFrames();

   SegmentDependentList<E, MovingReferenceFrame> getFootReferenceFrames();

   SegmentDependentList<E, MovingReferenceFrame> getSoleFrames();

   MovingReferenceFrame getPelvisFrame();

   MovingReferenceFrame getAnkleZUpFrame(E robotSegment);

   MovingReferenceFrame getFootFrame(E robotSegment);

   MovingReferenceFrame getLegJointFrame(E robotSegment, LegJointName legJointName);

   EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(E robotSegment);

   ReferenceFrame getIMUFrame();

   MovingReferenceFrame getPelvisZUpFrame();

   MovingReferenceFrame getSoleFrame(E robotSegment);

   MovingReferenceFrame getSoleZUpFrame(E robotSegment);

   SegmentDependentList<E, MovingReferenceFrame> getSoleZUpFrames();

   MovingReferenceFrame getChestFrame();

   MovingReferenceFrame getMidFootZUpGroundFrame();
}
