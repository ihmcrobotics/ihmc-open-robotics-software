package us.ihmc.robotics.quadruped;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public interface CommonQuadrupedReferenceFrames
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

   public abstract ReferenceFrame getCenterOfMassFrame();

   public abstract QuadrantDependentList<ReferenceFrame> getFootReferenceFrames();

}


