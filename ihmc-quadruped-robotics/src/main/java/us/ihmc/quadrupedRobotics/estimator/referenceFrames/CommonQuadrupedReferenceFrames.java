package us.ihmc.quadrupedRobotics.estimator.referenceFrames;

import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.geometry.QuadrupedGeometryTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public abstract class CommonQuadrupedReferenceFrames implements ReferenceFrames
{
   public abstract ReferenceFrame getBodyFrame();

   public abstract ReferenceFrame getBodyZUpFrame();

   public abstract ReferenceFrame getSideDependentMidFeetZUpFrame(RobotSide robotSide);

   public abstract ReferenceFrame getRootJointFrame();

   public abstract ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getFootFrame(RobotQuadrant robotQuadrant);
   
   public abstract ReferenceFrame getCenterOfFourHipsFrame();

   public abstract ReferenceFrame getCenterOfMassFrame();

   public abstract ReferenceFrame getCenterOfMassZUpFrame();

   public abstract QuadrantDependentList<ReferenceFrame> getFootReferenceFrames();
   
   public abstract ReferenceFrame getFrameBeforeLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName);

   private final QuadrantDependentList<Double> legLengths = new QuadrantDependentList<>();

   public void initializeCommonValues()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         legLengths.set(robotQuadrant, QuadrupedGeometryTools.calculateLegLength(this, robotQuadrant));
      }
   }

   public double getLegLength(RobotQuadrant robotQuadrant)
   {
      if (legLengths.get(robotQuadrant) == null)
         throw new RuntimeException("Call initializeCommonValues in your implementing reference frames class.");
      
      return legLengths.get(robotQuadrant);
   }

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
