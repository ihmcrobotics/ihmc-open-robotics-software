package us.ihmc.quadrupedRobotics.estimator.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.geometry.QuadrupedGeometryTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonQuadrupedReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public abstract class AbstractQuadrupedReferenceFrames implements CommonQuadrupedReferenceFrames
{
   public abstract ReferenceFrame getBodyFrame();

   public abstract ReferenceFrame getBodyZUpFrame();

   public abstract ReferenceFrame getSideDependentMidFeetZUpFrame(RobotSide robotSide);

   public abstract ReferenceFrame getRootJointFrame();

   public abstract ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant);

   public abstract ReferenceFrame getCenterOfFourHipsFrame();

   public abstract ReferenceFrame getCenterOfMassZUpFrame();

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

}
