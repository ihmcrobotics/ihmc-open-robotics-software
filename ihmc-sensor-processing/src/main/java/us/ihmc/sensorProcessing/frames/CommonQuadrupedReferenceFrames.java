package us.ihmc.sensorProcessing.frames;

import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public interface CommonQuadrupedReferenceFrames extends CommonLeggedReferenceFrames<RobotQuadrant>
{
   ReferenceFrame getBodyFrame();

   ReferenceFrame getBodyZUpFrame();

   ReferenceFrame getSideDependentMidFeetZUpFrame(RobotSide robotSide);

   ReferenceFrame getRootJointFrame();

   ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant);

   ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant);

   ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant);

   ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant);

   ReferenceFrame getCenterOfFourHipsFrame();

   ReferenceFrame getCenterOfMassZUpFrame();

   ReferenceFrame getFrameBeforeLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName);

   double getLegLength(RobotQuadrant robotQuadrant);

//   public abstract ReferenceFrame getMidTrotLineZUpFrame(RobotQuadrant quadrantAssocaitedWithTrotLine);
   
   /**
    * returns the center of the polygon made up using the provided robot quadrants, 
    * averaging the lowest front and the lowest hind Z values, 
    * and using the nominal yaw, pitch, and roll
    * @param feetQuadrants, feet 
    */
   public abstract ReferenceFrame getTripleSupportFrameAveragingLowestZHeightsAcrossEnds(RobotQuadrant footToExclude);

   /**
    * returns the center of the polygon made up using the provided robot quadrants, 
    * averaging the lowest front and the lowest hind Z values, 
    * and using the nominal yaw
    */
//   public abstract ReferenceFrame getZUpTripleSupportFrameAveragingLowestZHeightsAcrossEnds(RobotQuadrant footToExclude);

   /**
    * returns the center of the polygon made up using the four feet, 
    * averaging the lowest front and the lowest hind Z values, 
    * and using the nominal yaw, pitch, and roll
    */
   public abstract ReferenceFrame getCenterOfFeetFrameAveragingLowestZHeightsAcrossEnds();
   
   /**
    * returns the center of the four foot polygon, 
    * averaging the lowest front and the lowest hind Z values, 
    * and using the nominal yaw
    */
   public abstract ReferenceFrame getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

}
