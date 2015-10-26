package us.ihmc.quadrupedRobotics.footstepChooser;

import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class MidFootZUpSwingTargetGenerator implements SwingTargetGenerator
{
   private final double MINIMUM_VELOCITY_FOR_FULL_SKEW = 0.1;
   public static double MINIMUM_DISTANCE_FROM_SAMESIDE_FOOT = 0.04;
   public static double DEFAULT_STRIDE_LENGTH = 1.373;//0.34;
   public static double DEFAULT_STANCE_WIDTH = 0.36922*2;//0.24;
   public static double DEFAULT_MAX_SKEW = 0.1;
   public static double DEFAULT_MAX_YAW = 0.25;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonQuadrupedReferenceFrames referenceFrames;
   private final QuadrantDependentList<FramePoint> feetLocations = new QuadrantDependentList<FramePoint>();
   private final DoubleYoVariable minimumVelocityForFullSkew = new DoubleYoVariable("minimumVelocityForFullSkew", registry);
   private final DoubleYoVariable strideLength = new DoubleYoVariable("strideLength", registry);
   private final DoubleYoVariable stanceWidth = new DoubleYoVariable("stanceWidth", registry);
   private final DoubleYoVariable maxSkew = new DoubleYoVariable("maxSkew", registry);
   private final DoubleYoVariable maxYawPerStep = new DoubleYoVariable("maxYawPerStep", registry);
   private final DoubleYoVariable minimumDistanceFromSameSideFoot = new DoubleYoVariable("minimumDistanceFromSameSideFoot", registry);
   private final QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
   private final FramePoint centroid = new FramePoint(ReferenceFrame.getWorldFrame());

   public MidFootZUpSwingTargetGenerator(SwingTargetGeneratorParameters footStepParameters, CommonQuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footPosition = new FramePoint(ReferenceFrame.getWorldFrame());
         feetLocations.put(robotQuadrant, footPosition);
      }
      
      if(footStepParameters != null)
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

      RobotQuadrant sameSideQuadrant = swingLeg.getSameSideQuadrant();
      RobotSide swingSide = swingLeg.getSide();
      RobotSide oppositeSide = swingLeg.getOppositeSide();
      RobotEnd robotEnd = swingLeg.getEnd();

      ReferenceFrame oppositeSideZUpFrame = referenceFrames.getSideDependentMidFeetZUpFrame(oppositeSide);

      FramePoint swingFootPosition = feetLocations.get(swingLeg);
      FramePoint footPositionSameSideOppositeEnd = feetLocations.get(sameSideQuadrant);

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
         FramePoint footPosition = feetLocations.get(robotQuadrant);
         footPosition.setToZero(referenceFrames.getFootFrame(robotQuadrant));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         supportPolygon.setFootstep(robotQuadrant, footPosition);
      }
   }

   @Override
   public void getSwingTarget(QuadrupedSupportPolygon footPostions, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, FramePoint swingTargetToPack,
         double desiredYawRate)
   {
      // TODO Auto-generated method stub
      
   }
}
