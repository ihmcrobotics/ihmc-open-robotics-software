package us.ihmc.quadrupedRobotics.referenceFrames;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class CommonQuadrupedReferenceFrames
{
   public abstract ReferenceFrame getBodyFrame();

   public abstract ReferenceFrame getBodyZUpFrame();

   public abstract ReferenceFrame getSideDependentMidFeetZUpFrame(RobotSide robotSide);

   public abstract ReferenceFrame getEndDependentMidFeetZUpFrame(RobotEnd robotEnd);

   public abstract ReferenceFrame getRootJointFrame();

   public abstract ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getFootFrame(RobotQuadrant robotQuadrant);
   
   public abstract ReferenceFrame getCenterOfFourHipsFrame();

   public abstract ReferenceFrame getCenterOfMassFrame();

   public abstract QuadrantDependentList<ReferenceFrame> getFootReferenceFrames();
   
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

   public abstract ReferenceFrame getMidTrotLnieZUpFrame(RobotQuadrant quadrantAssocaitedWithTrotLine);
}
