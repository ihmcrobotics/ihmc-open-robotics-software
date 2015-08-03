package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorOutputData;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public class GroundReactionWrenchDistributorAchievedWrenchCalculator
{
   public static SpatialForceVector computeAchievedWrench(GroundReactionWrenchDistributorOutputData distributedWrench, ReferenceFrame expressedInFrame,
           List<PlaneContactState> contactStates)
   {
      FrameVector totalForce = new FrameVector(expressedInFrame);
      FrameVector totalMoment = new FrameVector(expressedInFrame);

      for (PlaneContactState planeContactState : contactStates)
      {
         FrameVector contactForce = new FrameVector(distributedWrench.getForce(planeContactState));
         contactForce.changeFrame(expressedInFrame);
         totalForce.add(contactForce);

         double normalTorqueMagnitude = distributedWrench.getNormalTorque(planeContactState);
         FrameVector normalTorque = new FrameVector(planeContactState.getPlaneFrame(), 0.0, 0.0, normalTorqueMagnitude);

         FramePoint2d centerOfPressure2d = distributedWrench.getCenterOfPressure(planeContactState);

         FramePoint centerOfPressure3d = new FramePoint(centerOfPressure2d.getReferenceFrame());
         centerOfPressure3d.setXY(centerOfPressure2d);
         centerOfPressure3d.changeFrame(expressedInFrame);

         FrameVector copToCoMVector = new FrameVector(expressedInFrame);
         copToCoMVector.changeFrame(expressedInFrame);
         copToCoMVector.sub(centerOfPressure3d);

         FrameVector crossProductTorque = new FrameVector(expressedInFrame);
         crossProductTorque.cross(contactForce, copToCoMVector);

         totalMoment.add(crossProductTorque);
         FrameVector normalTorqueInExpressedFrame = new FrameVector(normalTorque);
         normalTorqueInExpressedFrame.changeFrame(expressedInFrame);
         totalMoment.add(normalTorqueInExpressedFrame);
      }

      SpatialForceVector achievedWrench = new SpatialForceVector(expressedInFrame, totalForce.getVector(), totalMoment.getVector());

      return achievedWrench;
   }
}
