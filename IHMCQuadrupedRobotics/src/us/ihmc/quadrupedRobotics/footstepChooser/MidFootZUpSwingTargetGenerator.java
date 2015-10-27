package us.ihmc.quadrupedRobotics.footstepChooser;

import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class MidFootZUpSwingTargetGenerator implements SwingTargetGenerator
{
   private final double MINIMUM_VELOCITY_FOR_FULL_SKEW = 0.1;
   public static double MINIMUM_DISTANCE_FROM_SAMESIDE_FOOT = 0.04;
   public static double DEFAULT_STRIDE_LENGTH = 1.373;//0.34;
   public static double DEFAULT_STANCE_WIDTH = 0.36922 * 2;//0.24;
   public static double DEFAULT_MAX_SKEW = 0.1;
   public static double DEFAULT_MAX_YAW = 0.25;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonQuadrupedReferenceFrames referenceFrames;
   private final DoubleYoVariable minimumVelocityForFullSkew = new DoubleYoVariable("minimumVelocityForFullSkew", registry);
   private final DoubleYoVariable strideLength = new DoubleYoVariable("strideLength", registry);
   private final DoubleYoVariable stanceWidth = new DoubleYoVariable("stanceWidth", registry);
   private final DoubleYoVariable maxSkew = new DoubleYoVariable("maxSkew", registry);
   private final DoubleYoVariable maxYawPerStep = new DoubleYoVariable("maxYawPerStep", registry);
   private final DoubleYoVariable minimumDistanceFromSameSideFoot = new DoubleYoVariable("minimumDistanceFromSameSideFoot", registry);
   private final QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
   private final FramePoint centroid = new FramePoint(ReferenceFrame.getWorldFrame());

   public MidFootZUpSwingTargetGenerator(SwingTargetGeneratorParameters footStepParameters, CommonQuadrupedReferenceFrames referenceFrames,
         YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);

      if (footStepParameters != null)
      {
         minimumDistanceFromSameSideFoot.set(footStepParameters.getMinimumDistanceFromSameSideFoot());
         minimumVelocityForFullSkew.set(footStepParameters.getMinimumVelocityForFullSkew());
         strideLength.set(footStepParameters.getStrideLength());
         stanceWidth.set(footStepParameters.getStanceWidth());
         maxSkew.set(footStepParameters.getMaxSkew());
         maxYawPerStep.set(footStepParameters.getMaxYawPerStep());
      }
      else
      {
         minimumDistanceFromSameSideFoot.set(MINIMUM_DISTANCE_FROM_SAMESIDE_FOOT);
         minimumVelocityForFullSkew.set(MINIMUM_VELOCITY_FOR_FULL_SKEW);
         strideLength.set(DEFAULT_STRIDE_LENGTH);
         stanceWidth.set(DEFAULT_STANCE_WIDTH);
         maxSkew.set(DEFAULT_MAX_SKEW);
         maxYawPerStep.set(DEFAULT_MAX_YAW);
      }
   }

   @Override
   public void getSwingTarget(RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, FramePoint swingTargetToPack, double desiredYawRate)
   {
      updateFeetPositions();
      RobotSide oppositeSide = swingLeg.getOppositeSide();
      ReferenceFrame oppositeSideZUpFrame = referenceFrames.getSideDependentMidFeetZUpFrame(oppositeSide);
      calculateSwingTarget(supportPolygon, oppositeSideZUpFrame, swingLeg, desiredBodyVelocity, desiredYawRate, swingTargetToPack);
   }

   private TranslationReferenceFrame hindFoot = new TranslationReferenceFrame("backFoot", ReferenceFrame.getWorldFrame());
   private TranslationReferenceFrame frontFoot = new TranslationReferenceFrame("frontFoot", ReferenceFrame.getWorldFrame());
   private MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame("MidFeetZUpFrame", ReferenceFrame.getWorldFrame(), hindFoot, frontFoot);
   
   @Override
   public void getSwingTarget(QuadrupedSupportPolygon footPostions, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, FramePoint swingTargetToPack,
         double desiredYawRate)
   {
      FramePoint across = footPostions.getFootstep(swingLeg.getAcrossBodyQuadrant());
      FramePoint diag = footPostions.getFootstep(swingLeg.getDiagonalOppositeQuadrant());
      
      if(swingLeg.isQuadrantInFront())
      {
         frontFoot.updateTranslation(across);
         hindFoot.updateTranslation(diag);
      }
      else
      {
         frontFoot.updateTranslation(diag);
         hindFoot.updateTranslation(across);
      }
      midFeetZUpFrame.update();
      
      calculateSwingTarget(footPostions, midFeetZUpFrame, swingLeg, desiredBodyVelocity, desiredYawRate, swingTargetToPack);
   }

   private void calculateSwingTarget(QuadrupedSupportPolygon supportPolygon, ReferenceFrame oppositeSideZUpFrame, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, double desiredYawRate,
         FramePoint swingTargetToPack)
   {
      RobotQuadrant sameSideQuadrant = swingLeg.getSameSideQuadrant();
      RobotSide swingSide = swingLeg.getSide();
      RobotEnd robotEnd = swingLeg.getEnd();

      FramePoint swingFootPosition = new FramePoint(supportPolygon.getFootstep(swingLeg));
      FramePoint footPositionSameSideOppositeEnd = new FramePoint(supportPolygon.getFootstep(sameSideQuadrant));

      //midZUpFrame is oriented so X is perpendicular to the two same side feet

      //handle forward backward placement
      swingFootPosition.changeFrame(oppositeSideZUpFrame);
      double halfStrideLength = 0.5 * strideLength.getDoubleValue();
      double clippedSkew = MathTools.clipToMinMax(maxSkew.getDoubleValue(), 0.0, halfStrideLength);
      double amountToSkew = MathTools.clipToMinMax(desiredBodyVelocity.getX() / minimumVelocityForFullSkew.getDoubleValue(), 1.0) * clippedSkew;
      double newY = robotEnd.negateIfFrontEnd(halfStrideLength) - amountToSkew;
      swingFootPosition.setY(newY);

      //handle left right placement
      swingFootPosition.setX(swingSide.negateIfRightSide(stanceWidth.getDoubleValue()));

      // maintain minimumDistanceFromSameSideFoot inline
      footPositionSameSideOppositeEnd.changeFrame(oppositeSideZUpFrame);
      double minimumRadiusFromSameSideFoot = minimumDistanceFromSameSideFoot.getDoubleValue();

      boolean footIsForwardOfOtherFoot = swingFootPosition.getY() < footPositionSameSideOppositeEnd.getY();
      boolean footIsBehindOtherFoot = swingFootPosition.getY() > footPositionSameSideOppositeEnd.getY();
      boolean footIsCloseToOtherFoot = swingFootPosition.distance(footPositionSameSideOppositeEnd) < minimumRadiusFromSameSideFoot;

      if ((robotEnd.equals(RobotEnd.HIND) && footIsForwardOfOtherFoot) || (robotEnd.equals(RobotEnd.FRONT) && footIsBehindOtherFoot) || footIsCloseToOtherFoot)
      {
         swingFootPosition.setY(footPositionSameSideOppositeEnd.getY());
         swingFootPosition.add(0.0, robotEnd.negateIfFrontEnd(minimumRadiusFromSameSideFoot), 0.0);
      }

      swingFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

      //rotate the foot about the centroid of the predicted foot polygon
      supportPolygon.setFootstep(swingLeg, swingFootPosition);
      supportPolygon.getCentroid(centroid);
      double deltaYaw = MathTools.clipToMinMax(desiredYawRate, maxYawPerStep.getDoubleValue());
      swingFootPosition.set(swingFootPosition.yawAboutPoint(centroid, deltaYaw));
      //      swingTargetToPack.setZ(0.0);
      swingTargetToPack.set(swingFootPosition);
   }

   private void updateFeetPositions()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footPosition = supportPolygon.getFootstep(robotQuadrant);
         if(footPosition == null)
         {
            footPosition = new FramePoint(ReferenceFrame.getWorldFrame());
         }
         
         footPosition.setToZero(referenceFrames.getFootFrame(robotQuadrant));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         supportPolygon.setFootstep(robotQuadrant, footPosition);
      }
   }
}
