package us.ihmc.quadrupedRobotics.util;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedLinkLengths
{
   public QuadrantDependentList<MutableDouble> shinLengths = new QuadrantDependentList<MutableDouble>();
   public QuadrantDependentList<MutableDouble> thighLengths = new QuadrantDependentList<MutableDouble>();
   public QuadrantDependentList<MutableDouble> hipLengths = new QuadrantDependentList<MutableDouble>();
   
   public QuadrupedLinkLengths(CommonQuadrupedReferenceFrames quadrupedReferenceFrames)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame legAttachmentFrame = quadrupedReferenceFrames.getLegAttachmentFrame(robotQuadrant);
         ReferenceFrame beforeHipPitchFrame = quadrupedReferenceFrames.getFrameBeforeLegJoint(robotQuadrant, LegJointName.HIP_PITCH);
         ReferenceFrame hipPitchFrame = quadrupedReferenceFrames.getHipPitchFrame(robotQuadrant);
         ReferenceFrame beforeKneePitchFrame = quadrupedReferenceFrames.getFrameBeforeLegJoint(robotQuadrant, LegJointName.KNEE);
         ReferenceFrame kneePitchFrame = quadrupedReferenceFrames.getKneeFrame(robotQuadrant);
         ReferenceFrame footFrame = quadrupedReferenceFrames.getFootFrame(robotQuadrant);

         FramePoint legAttachment = new FramePoint(legAttachmentFrame);
         FramePoint beforeHipPitch = new FramePoint(beforeHipPitchFrame);
         FramePoint hipPitch = new FramePoint(hipPitchFrame);
         FramePoint beforeKneePitch = new FramePoint(beforeKneePitchFrame);
         FramePoint kneePitch = new FramePoint(kneePitchFrame);
         FramePoint foot = new FramePoint(footFrame);

         beforeHipPitch.changeFrame(legAttachmentFrame);
         double hipLength = legAttachment.distance(beforeHipPitch);
         hipLengths.put(robotQuadrant, new MutableDouble(hipLength));

         beforeKneePitch.changeFrame(hipPitchFrame);
         double thighLength = hipPitch.distance(beforeKneePitch);
         thighLengths.put(robotQuadrant, new MutableDouble(thighLength));

         foot.changeFrame(kneePitchFrame);
         double shinLength = kneePitch.distance(foot);
         shinLengths.put(robotQuadrant, new MutableDouble(shinLength));
      }
   }
   
   public double getHipLength(RobotQuadrant robotQuadrant)
   {
      return hipLengths.get(robotQuadrant).doubleValue();
   }
   
   public double getThighLength(RobotQuadrant robotQuadrant)
   {
      return thighLengths.get(robotQuadrant).doubleValue();
   }
   
   public double getShinLength(RobotQuadrant robotQuadrant)
   {
      return shinLengths.get(robotQuadrant).doubleValue();
   }
}
