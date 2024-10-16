package us.ihmc.sensorProcessing.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.commons.robotics.robotSide.RobotSegment;
import us.ihmc.commons.robotics.robotSide.SegmentDependentList;

import java.util.EnumMap;

public interface CommonLeggedReferenceFrames<E extends Enum<E> & RobotSegment<E>> extends ReferenceFrames
{
   SegmentDependentList<E, MovingReferenceFrame> getAnkleZUpReferenceFrames();

   SegmentDependentList<E, MovingReferenceFrame> getFootReferenceFrames();

   SegmentDependentList<E, MovingReferenceFrame> getSoleFrames();

   MovingReferenceFrame getAnkleZUpFrame(E robotSegment);

   MovingReferenceFrame getFootFrame(E robotSegment);

   MovingReferenceFrame getLegJointFrame(E robotSegment, LegJointName legJointName);

   EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(E robotSegment);

   ReferenceFrame getIMUFrame();

   MovingReferenceFrame getSoleFrame(E robotSegment);

   MovingReferenceFrame getSoleZUpFrame(E robotSegment);

   SegmentDependentList<E, MovingReferenceFrame> getSoleZUpFrames();

}
