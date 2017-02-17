package us.ihmc.quadrupedRobotics.geometry;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.shapes.FrameEllipsoid3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedGeometryTools
{
   // ONE-OFF METHODS
   
   /**
    * Calculates the leg length of the quadruped.
    * <p>
    * Not real-time.
    * 
    * @param referenceFrames
    * @param robotQuadrant
    * @return legLength
    */
   public static double calculateLegLength(CommonQuadrupedReferenceFrames referenceFrames, RobotQuadrant robotQuadrant)
   {
      ReferenceFrame hipPitchFrame = referenceFrames.getHipPitchFrame(robotQuadrant);
      ReferenceFrame kneePitchFrame = referenceFrames.getKneeFrame(robotQuadrant);
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);

      FramePoint hipPitch = new FramePoint(hipPitchFrame);
      FramePoint kneePitch = new FramePoint(kneePitchFrame);
      FramePoint foot = new FramePoint(footFrame);

      kneePitch.changeFrame(hipPitchFrame);
      double thighLength = kneePitch.distance(hipPitch);

      kneePitch.changeFrame(kneePitchFrame);
      foot.changeFrame(kneePitchFrame);
      double shinLength = foot.distance(kneePitch);

      return thighLength + shinLength;
   }
   
   // REAL-TIME METHODS
   
   /**
    * Updates the feasible step ellipsoid. Assuming projected circles on z=0 for now.
    * <p>
    * Real-time safe, this method keeps track of the available step region for leg of a quadruped robot.
    * 
    * @param referenceFrames
    * @param quadrantHipPitchPoint
    * @param ellipsoidToPackInHipPitchFrame
    * @param robotQuadrant
    */
   public static void updateFootstepWorkspace(RobotQuadrant robotQuadrant, FrameEllipsoid3d ellipsoidToPack, CommonQuadrupedReferenceFrames referenceFrames)
   {
      double hipPitchHeight = ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(referenceFrames.getHipPitchFrame(robotQuadrant)).getM23();
      double maxStepDistance = Math.sqrt(Math.pow(referenceFrames.getLegLength(robotQuadrant), 2) - Math.pow(hipPitchHeight, 2));
      
      ellipsoidToPack.checkReferenceFrameMatch(referenceFrames.getHipPitchFrame(robotQuadrant));
      
      RigidBodyTransform ellipsoidTransform = new RigidBodyTransform();
      ellipsoidTransform.setTranslation(0.0, 0.0, -hipPitchHeight);
      ellipsoidToPack.applyTransform(ellipsoidTransform);
      ellipsoidToPack.getEllipsoid3d().setXRadius(maxStepDistance);
      ellipsoidToPack.getEllipsoid3d().setYRadius(maxStepDistance);
      ellipsoidToPack.getEllipsoid3d().setZRadius(0.0);
   }
}
