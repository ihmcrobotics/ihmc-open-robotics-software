package us.ihmc.darpaRoboticsChallenge.footstepGenerator;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepValidityMetric;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.vecmath.Vector3d;

/**
 * Created by agrabertilton on 2/20/15.
 */
public class BasicFootstepValidityMetric implements FootstepValidityMetric
{
   private static final double EPSILON = 1e-13;
   SteppingParameters steppingParameters;

   public BasicFootstepValidityMetric(SteppingParameters steppingParameters)
   {
      this.steppingParameters = steppingParameters;
   }

   @Override
   public boolean footstepValid(FootstepData stanceFootstep, FootstepData prospectiveFootstep)
   {
      if (stanceFootstep.getRobotSide() == prospectiveFootstep.getRobotSide()) return false;
      double stanceYaw = RotationFunctions.getYawFromQuaternion(stanceFootstep.getOrientation());
      double yawDiff = RotationFunctions.getYawFromQuaternion(prospectiveFootstep.getOrientation()) - stanceYaw;

      yawDiff = AngleTools.trimAngleMinusPiToPi(yawDiff);
      double sign = (prospectiveFootstep.getRobotSide() == RobotSide.LEFT) ? 1.0 : -1.0;
      yawDiff *= sign;

      if (yawDiff > steppingParameters.getMaxAngleTurnOutwards() + EPSILON || (-1.0 * yawDiff > steppingParameters.getMaxAngleTurnInwards() + EPSILON))
      {
         return false;
      }

      Vector3d stepVector = new Vector3d(prospectiveFootstep.getLocation());
      stepVector.sub(stanceFootstep.getLocation());
      double stepLengthInStanceFrame = Math.cos(stanceYaw) * stepVector.x + Math.sin(stanceYaw) * stepVector.y;
      double stepWidthInStanceFrame = Math.cos(stanceYaw) * stepVector.y - Math.sin(stanceYaw) * stepVector.x;
      stepWidthInStanceFrame *= sign;
      double zDiff = stepVector.z;
      if (zDiff > steppingParameters.getMaxStepUp() + EPSILON || -1*zDiff + EPSILON > steppingParameters.getMaxStepDown()){
         return false;
      }
      if (stepWidthInStanceFrame < steppingParameters.getMinStepWidth() + EPSILON){
         return false;
      }

      boolean stricterBoundaries = false;
      double thresholdFactor = 1.0/3.0;
      if (zDiff > steppingParameters.getMaxStepUp() * thresholdFactor + EPSILON|| -1*zDiff + EPSILON> steppingParameters.getMaxStepDown() * thresholdFactor){
         stricterBoundaries = true;
      }

      if (!checkArea(prospectiveFootstep, steppingParameters.getMinAreaPercentForValidFootstep(), steppingParameters)){
         return false;
      }

      if (!checkArea(stanceFootstep, steppingParameters.getDangerAreaPercentForValidFootstep(), steppingParameters) ||
            !checkArea(prospectiveFootstep, steppingParameters.getDangerAreaPercentForValidFootstep(), steppingParameters))
      {
         stricterBoundaries = true;
      }

      double scaleFactor = stricterBoundaries? 2.0/3.0 : 1.0;
      double backwardScale = 0.5;
      if (stepWidthInStanceFrame > steppingParameters.getMaxStepWidth() * scaleFactor + EPSILON){ return false;}
      if (stepLengthInStanceFrame > steppingParameters.getMaxStepLength() * scaleFactor + EPSILON){ return false;}
      if (-1.0 * stepLengthInStanceFrame > steppingParameters.getMaxStepLength() * scaleFactor * backwardScale + EPSILON){ return false;}

      //boundary lines y = Ax+B
      //A = (y2 - y1) / (x2 - x1)
      double A;
      //B = y2 - A*x2
      double B;
      //y < Ax + B (maxStepLength*scaleFactor, defaultWidth), (0.0, maxStepWidth*scaleFactor)
      A = (steppingParameters.getMaxStepWidth() * scaleFactor - steppingParameters.getInPlaceWidth()) / (0 - steppingParameters.getMaxStepLength() * scaleFactor);
      B = (steppingParameters.getMaxStepWidth() * scaleFactor);
      if (stepWidthInStanceFrame - (A * stepLengthInStanceFrame + B) > EPSILON){ return false;}

      //y < Ax + B (-1 * backwardScale * maxStepLength*scaleFactor, defaultWidth), (0.0, maxStepWidth*scaleFactor)
      A = (steppingParameters.getMaxStepWidth() * scaleFactor - steppingParameters.getInPlaceWidth())/(0 - -1* backwardScale * steppingParameters.getMaxStepLength() * scaleFactor);
      B = (steppingParameters.getMaxStepWidth() * scaleFactor);
      if (stepWidthInStanceFrame - (A * stepLengthInStanceFrame + B) > EPSILON){ return false;}

      return true;
   }

   @Override
   public boolean footstepValid(FootstepData previousFootstep, FootstepData stanceFootstep, FootstepData prospectiveFootstep)
   {
      if (previousFootstep.getRobotSide() != prospectiveFootstep.getRobotSide()) return false;
      if (!footstepValid(stanceFootstep, prospectiveFootstep)){
         return false;
      }
      //TODO @Agraber make this more comprehensive
      double stanceYaw = RotationFunctions.getYawFromQuaternion(stanceFootstep.getOrientation());
      double sign = (prospectiveFootstep.getRobotSide() == RobotSide.LEFT) ? 1.0 : -1.0;

      Vector3d stepVector = new Vector3d(prospectiveFootstep.getLocation());
      stepVector.sub(stanceFootstep.getLocation());
      double stepLengthInStanceFrame = Math.cos(stanceYaw) * stepVector.x + Math.sin(stanceYaw) * stepVector.y;
      double stepWidthInStanceFrame = Math.cos(stanceYaw) * stepVector.y - Math.sin(stanceYaw) * stepVector.x;
      stepWidthInStanceFrame *= sign;
      double zDiff = stepVector.z;
      double swingHeightInRelationToStanceFoot = previousFootstep.getLocation().getZ() + prospectiveFootstep.getSwingHeight() - stanceFootstep.getLocation().getZ();

      boolean stricterBoundaries = false;
      if (swingHeightInRelationToStanceFoot > steppingParameters.getMaxStepUp() * 0.5){
         stricterBoundaries = true;
      }

      double scaleFactor = stricterBoundaries? 2.0/3.0 : 1.0;
      double backwardScale = 0.5;
      if (stepWidthInStanceFrame > steppingParameters.getMaxStepWidth() * scaleFactor + EPSILON){ return false;}
      if (stepLengthInStanceFrame > steppingParameters.getMaxStepLength() * scaleFactor + EPSILON){ return false;}
      if (-1.0 * stepLengthInStanceFrame > steppingParameters.getMaxStepLength() * scaleFactor * backwardScale + EPSILON){ return false;}

      //boundary lines y = Ax+B
      //A = (y2 - y1) / (x2 - x1)
      double A;
      //B = y2 - A*x2
      double B;
      //y < Ax + B (maxStepLength*scaleFactor, defaultWidth), (0.0, maxStepWidth*scaleFactor)
      A = (steppingParameters.getMaxStepWidth() * scaleFactor - steppingParameters.getInPlaceWidth()) / (0 - steppingParameters.getMaxStepLength() * scaleFactor);
      B = (steppingParameters.getMaxStepWidth() * scaleFactor);
      if (stepWidthInStanceFrame - (A * stepLengthInStanceFrame + B) > EPSILON){ return false;}

      //y < Ax + B (-1 * backwardScale * maxStepLength*scaleFactor, defaultWidth), (0.0, maxStepWidth*scaleFactor)
      A = (steppingParameters.getMaxStepWidth() * scaleFactor - steppingParameters.getInPlaceWidth())/(0 - -1* backwardScale * steppingParameters.getMaxStepLength() * scaleFactor);
      B = (steppingParameters.getMaxStepWidth() * scaleFactor);
      if (stepWidthInStanceFrame - (A * stepLengthInStanceFrame + B) > EPSILON){ return false;}

      return true;
   }

   private boolean checkArea(FootstepData footstep, double percentageToCheckAgainst, SteppingParameters parameters){
      if (footstep.predictedContactPoints == null) return true;

      ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(footstep.getPredictedContactPoints());
      convexPolygon2d.update();
      double area = convexPolygon2d.getArea();
      double percentage = area / parameters.getFootstepArea();
      if (percentage + EPSILON < percentageToCheckAgainst) return false;
      return true;
   }
}
