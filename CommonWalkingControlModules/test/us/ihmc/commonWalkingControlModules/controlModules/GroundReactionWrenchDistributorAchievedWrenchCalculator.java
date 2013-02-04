package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class GroundReactionWrenchDistributorAchievedWrenchCalculator
{

   public static SpatialForceVector computeAchievedWrench(GroundReactionWrenchDistributor distributor, ReferenceFrame expressedInFrame, List<PlaneContactState> contactStates)
   {
      FrameVector totalForce = new FrameVector(expressedInFrame);
      FrameVector totalMoment = new FrameVector(expressedInFrame);

      GroundReactionWrenchDistributorOutputData distributedWrench = distributor.getSolution();
      
      for (PlaneContactState planeContactState : contactStates)
      {
         FrameVector contactForce = distributedWrench.getForce(planeContactState).changeFrameCopy(expressedInFrame);
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
         totalMoment.add(normalTorque.changeFrameCopy(expressedInFrame));
      }
      
      SpatialForceVector achievedWrench = new SpatialForceVector(expressedInFrame, totalForce.getVector(), totalMoment.getVector());
      return achievedWrench;
   }
}
