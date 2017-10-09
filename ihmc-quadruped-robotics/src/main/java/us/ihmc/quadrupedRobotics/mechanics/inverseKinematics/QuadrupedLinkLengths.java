package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
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
         ReferenceFrame beforeKneePitchFrame = quadrupedReferenceFrames.getFrameBeforeLegJoint(robotQuadrant, LegJointName.KNEE_PITCH);
         ReferenceFrame kneePitchFrame = quadrupedReferenceFrames.getKneeFrame(robotQuadrant);
         ReferenceFrame footFrame = quadrupedReferenceFrames.getFootFrame(robotQuadrant);

         FramePoint3D legAttachment = new FramePoint3D(legAttachmentFrame);
         FramePoint3D beforeHipPitch = new FramePoint3D(beforeHipPitchFrame);
         FramePoint3D hipPitch = new FramePoint3D(hipPitchFrame);
         FramePoint3D beforeKneePitch = new FramePoint3D(beforeKneePitchFrame);
         FramePoint3D kneePitch = new FramePoint3D(kneePitchFrame);
         FramePoint3D foot = new FramePoint3D(footFrame);

         beforeHipPitch.changeFrame(legAttachmentFrame);
         double hipLength = legAttachment.distance(beforeHipPitch);
         hipLengths.set(robotQuadrant, new MutableDouble(hipLength));

         beforeKneePitch.changeFrame(hipPitchFrame);
         double thighLength = hipPitch.distance(beforeKneePitch);
         thighLengths.set(robotQuadrant, new MutableDouble(thighLength));

         foot.changeFrame(kneePitchFrame);
         double shinLength = kneePitch.distance(foot);
         shinLengths.set(robotQuadrant, new MutableDouble(shinLength));
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
