package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule.LegConfigurationType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoDouble;

public class LegConfigurationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double forwardSteppingThreshold = -0.05;
   private static final double minimumAngleForSideStepping = 45.0;

   private static final double stepDownTooFar = -0.10;
   private static final double stepHeightForCollapse = 10.0;

   private final YoBoolean attemptToStraightenLegs = new YoBoolean("attemptToStraightenLegs", registry);

   private final YoDouble stepHeight = new YoDouble("stepHeight", registry);
   private final YoDouble maxStepHeightForCollapse = new YoDouble("maxStepHeightForCollapse", registry);
   private final YoDouble stepHeightForForcedCollapse = new YoDouble("stepHeightForForcedCollapsing", registry);
   private final YoDouble minStepLengthForCollapse = new YoDouble("minStepLengthForCollapse", registry);

   private final SideDependentList<LegConfigurationControlModule> legConfigurationControlModules = new SideDependentList<>();
   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final double inPlaceWidth;
   private final double footLength;

   public LegConfigurationManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                                  YoVariableRegistry parentRegistry)
   {
      this.feet = controllerToolbox.getContactableFeet();

      StraightLegWalkingParameters straightLegWalkingParameters = walkingControllerParameters.getStraightLegWalkingParameters();
      for (RobotSide robotSide : RobotSide.values)
         legConfigurationControlModules.put(robotSide, new LegConfigurationControlModule(robotSide, controllerToolbox, straightLegWalkingParameters, registry));

      attemptToStraightenLegs.set(straightLegWalkingParameters.attemptToStraightenLegs());

      this.inPlaceWidth = walkingControllerParameters.getInPlaceWidth();
      this.footLength = walkingControllerParameters.getFootBackwardOffset() + walkingControllerParameters.getFootForwardOffset();

      maxStepHeightForCollapse.set(stepHeightForCollapse);
      stepHeightForForcedCollapse.set(stepDownTooFar);
      minStepLengthForCollapse.set(walkingControllerParameters.getFootLength());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legConfigurationControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legConfigurationControlModules.get(robotSide).doControl();
      }
   }

   public void startSwing(RobotSide upcomingSwingSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         legConfigurationControlModules.get(upcomingSwingSide).setKneeAngleState(LegConfigurationType.BENT);
      }
   }

   public boolean isLegCollapsed(RobotSide robotSide)
   {
      LegConfigurationType legConfigurationControlState = legConfigurationControlModules.get(robotSide).getCurrentKneeControlState();
      return (legConfigurationControlState.equals(LegConfigurationType.BENT) || legConfigurationControlState.equals(LegConfigurationType.COLLAPSE));
   }

   public boolean isLegBent(RobotSide robotSide)
   {
      LegConfigurationType legConfigurationControlState = legConfigurationControlModules.get(robotSide).getCurrentKneeControlState();
      return (legConfigurationControlState.equals(LegConfigurationType.BENT));
   }

   public void collapseLegDuringTransfer(RobotSide transferSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         legConfigurationControlModules.get(transferSide.getOppositeSide()).setKneeAngleState(LegConfigurationType.BENT);
      }
   }

   public void collapseLegDuringSwing(RobotSide supportSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         legConfigurationControlModules.get(supportSide).setKneeAngleState(LegConfigurationType.COLLAPSE);
      }
   }

   public void straightenLegDuringSwing(RobotSide swingSide)
   {
      if (legConfigurationControlModules.get(swingSide).getCurrentKneeControlState() != LegConfigurationType.STRAIGHTEN &&
            legConfigurationControlModules.get(swingSide).getCurrentKneeControlState() != LegConfigurationType.STRAIGHT)
      {
         //beginStraightening(swingSide);
         setStraight(swingSide);
         setFullyExtendLeg(swingSide, true);

         boolean isNextStepTooLow = stepHeight.getDoubleValue() < stepHeightForForcedCollapse.getDoubleValue();
         if (isNextStepTooLow)
            setLegBracing(swingSide, true);
      }
   }

   public void setFullyExtendLeg(RobotSide robotSide, boolean fullyExtendLeg)
   {
      legConfigurationControlModules.get(robotSide).setFullyExtendLeg(fullyExtendLeg);
   }

   public void setLegBracing(RobotSide robotSide, boolean legBracing)
   {
      legConfigurationControlModules.get(robotSide).setLegBracing(legBracing);
   }

   public void setStraight(RobotSide robotSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         legConfigurationControlModules.get(robotSide).setKneeAngleState(LegConfigurationType.STRAIGHT);
      }
   }

   public void beginStraightening(RobotSide robotSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         legConfigurationControlModules.get(robotSide).setKneeAngleState(LegConfigurationType.STRAIGHTEN);
      }
   }

   private final FramePoint2d tempLeadingFootPosition = new FramePoint2d();
   private final FramePoint2d tempTrailingFootPosition = new FramePoint2d();
   private final FramePoint tempLeadingFootPositionInWorld = new FramePoint();
   private final FramePoint tempTrailingFootPositionInWorld = new FramePoint();

   public boolean areFeetWellPositionedForCollapse(RobotSide trailingLeg)
   {
      ReferenceFrame frontFootFrame = feet.get(trailingLeg.getOppositeSide()).getSoleFrame();
      return areFeetWellPositionedForCollapse(trailingLeg, frontFootFrame);
   }

   public boolean areFeetWellPositionedForCollapse(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getSoleFrame();
      tempTrailingFootPosition.setToZero(trailingFootFrame);
      tempLeadingFootPosition.setToZero(frontFootFrame);
      tempLeadingFootPosition.changeFrameAndProjectToXYPlane(trailingFootFrame);

      if (Math.abs(tempLeadingFootPosition.getY()) > inPlaceWidth)
         tempLeadingFootPosition.setY(tempLeadingFootPosition.getY() + trailingLeg.negateIfRightSide(inPlaceWidth));
      else
         tempLeadingFootPosition.setY(0.0);

      tempLeadingFootPositionInWorld.setToZero(frontFootFrame);
      tempTrailingFootPositionInWorld.setToZero(trailingFootFrame);
      tempLeadingFootPositionInWorld.changeFrame(worldFrame);
      tempTrailingFootPositionInWorld.changeFrame(worldFrame);

      stepHeight.set(tempLeadingFootPositionInWorld.getZ() - tempTrailingFootPositionInWorld.getZ());

      boolean isNextStepTooLow = stepHeight.getDoubleValue() < stepHeightForForcedCollapse.getDoubleValue();
      if (isNextStepTooLow)
         return true;

      boolean isForwardStepping = tempLeadingFootPositionInWorld.getX() > forwardSteppingThreshold;
      if (!isForwardStepping)
         return false;

      boolean isNextStepTooHigh = stepHeight.getDoubleValue() > maxStepHeightForCollapse.getDoubleValue();
      if (isNextStepTooHigh)
         return false;

      boolean isSideStepping = Math.abs(Math.atan2(tempLeadingFootPosition.getY(), tempLeadingFootPosition.getX())) > Math.toRadians(minimumAngleForSideStepping);
      if (isSideStepping)
         return false;

      boolean isStepLongEnough = tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForCollapse.getDoubleValue();
      boolean isStepLongEnoughAlongX = tempLeadingFootPosition.getX() > footLength;
      return isStepLongEnough && isStepLongEnoughAlongX;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      //return footControlModules.get(robotSide).getFeedbackControlCommand();
      return null;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return legConfigurationControlModules.get(robotSide).getInverseDynamicsCommand();
   }
}
