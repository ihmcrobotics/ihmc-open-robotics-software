package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;

/**
 * When the robot has a high upper body angular momentum the balancing algorithm is not aware about the
 * need to slow down the upper body eventually. This class can adjust the capture point plan taking into
 * account future slow down torques on the upper body.
 *
 * @author shadylady
 *
 */
public class AngularMomentumBasedStrategyAdjuster
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SlowDownAssumption assumption = SlowDownAssumption.INSTANTANEOUS_VELOCITY_CHANGE;
   private enum SlowDownAssumption {
      NONE,
      INSTANTANEOUS_VELOCITY_CHANGE
   }

   private final DoubleYoVariable gain = new DoubleYoVariable("MomentumGain", registry);
   private static final double defaultGain = 0.4;

   private final HighLevelHumanoidControllerToolbox toolbox;
   private final FrameVector upperBodyAngularMomentum = new FrameVector();
   private final MomentumCalculator upperBodyMomentumCalculator;
   private final YoFrameVector yoUpperBodyAngularMomentum;
   private final double upperBodyMass;

   private final FramePoint2d desiredICPPosition = new FramePoint2d();
   private final FrameVector2d desiredICPVelocity = new FrameVector2d();

   private final FramePoint2d naiveDesiredICPPosition = new FramePoint2d();
   private final FrameVector2d naiveDesiredICPVelocity = new FrameVector2d();

   public AngularMomentumBasedStrategyAdjuster(HighLevelHumanoidControllerToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;

      RigidBody chest = toolbox.getFullRobotModel().getChest();
      ReferenceFrame centerOfMassFrame = toolbox.getReferenceFrames().getCenterOfMassFrame();
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();

      upperBodyMomentumCalculator = new MomentumCalculator(twistCalculator, ScrewTools.computeSupportAndSubtreeSuccessors(twistCalculator.getRootBody()));
      yoUpperBodyAngularMomentum = new YoFrameVector("UpperBodyAngularMomentum", centerOfMassFrame, registry);
      upperBodyMass = TotalMassCalculator.computeSubTreeMass(chest);

      gain.set(defaultGain);
      parentRegistry.addChild(registry);
   }

   public void compute(FramePoint2d naiveDesiredICPPosition, FrameVector2d naiveDesiredICPVelocity)
   {
      this.naiveDesiredICPPosition.setIncludingFrame(naiveDesiredICPPosition);
      this.naiveDesiredICPVelocity.setIncludingFrame(naiveDesiredICPVelocity);

      switch (assumption)
      {
      case INSTANTANEOUS_VELOCITY_CHANGE:
         computeForInstantaneuousVelocityChange();
      case NONE:
      default:
         desiredICPPosition.setIncludingFrame(naiveDesiredICPPosition);
         desiredICPVelocity.setIncludingFrame(naiveDesiredICPVelocity);
      }
   }

   private void computeForInstantaneuousVelocityChange()
   {
      toolbox.getUpperBodyAngularMomentum(upperBodyAngularMomentum);

//      ReferenceFrame comFrame = upperBodyAngularMomentumXY.getReferenceFrame();
//      localDesiredCapturePoint.setIncludingFrame(desiredCapturePoint);
//      localDesiredCapturePoint.changeFrameAndProjectToXYPlane(comFrame);
//
//      double scaleFactor = momentumGain.getDoubleValue() * omega0.getDoubleValue() / (upperBodyMass * gravity);
//
//      adjustedDesiredCapturePoint.setIncludingFrame(comFrame, upperBodyAngularMomentumXY.getY(), upperBodyAngularMomentumXY.getX());
//      adjustedDesiredCapturePoint.scale(scaleFactor);
//      adjustedDesiredCapturePoint.add(localDesiredCapturePoint);
//      adjustedDesiredCapturePoint.changeFrameAndProjectToXYPlane(desiredCapturePoint.getReferenceFrame());
   }

   public void getDesiredICPPosition(FramePoint2d desiredPositionToPack)
   {
      desiredPositionToPack.setIncludingFrame(desiredICPPosition);
   }

   public void getDesiredICPVelocity(FrameVector2d desiredVelocityToPack)
   {
      desiredVelocityToPack.setIncludingFrame(desiredICPVelocity);
   }
}
