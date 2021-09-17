package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies.CustomCoMPositionPolicy;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies.CustomDCMPositionPolicy;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies.CustomMPCPolicy;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class JumpingHeuristics
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final JumpingControllerToolbox controllerToolbox;

   private final CustomCoMPositionPolicy takeoffCoMPolicy = new CustomCoMPositionPolicy();
   private final CustomCoMPositionPolicy touchdownCoMPolicy = new CustomCoMPositionPolicy();
   private final CustomDCMPositionPolicy touchdownDCMPolicy = new CustomDCMPositionPolicy();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble desiredWeightForStateChangeHeights = new YoDouble("desiredWeightForStateChangeHeights", registry);
   private final YoDouble maximumTouchdownDistance = new YoDouble("maximumTouchdownDistance", registry);
   private final YoDouble maxLegLength = new YoDouble("maxLegLength", registry);

   private final YoDouble touchdownHeight = new YoDouble("touchdownHeight", registry);
   private final YoDouble touchdownDistance = new YoDouble("touchdownDistance", registry);
   private final YoDouble takeOffHeight = new YoDouble("takeOffHeight", registry);
   private final YoDouble takeOffDistance = new YoDouble("takeOffDistance", registry);

   private final List<CustomMPCPolicy> policies = new ArrayList<>();

   private final FramePoint3D takeOffCoMPosition = new FramePoint3D();
   private final FramePoint3D touchdownCoMPosition = new FramePoint3D();
   private final FramePoint3D touchdownDCMPosition = new FramePoint3D();

   public JumpingHeuristics(JumpingControllerToolbox controllerToolbox, YoRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      desiredWeightForStateChangeHeights.set(5e2);
      maximumTouchdownDistance.set(0.25);
      maxLegLength.set(1.0);

      takeoffCoMPolicy.getSelectionMatrix().clearSelection();
      takeoffCoMPolicy.getSelectionMatrix().selectXAxis(true);
      takeoffCoMPolicy.getSelectionMatrix().selectZAxis(true);

      touchdownCoMPolicy.getSelectionMatrix().clearSelection();
      touchdownCoMPolicy.getSelectionMatrix().selectXAxis(true);
      touchdownCoMPolicy.getSelectionMatrix().selectZAxis(true);

      touchdownDCMPolicy.getSelectionMatrix().clearSelection();
      touchdownDCMPolicy.getSelectionMatrix().selectXAxis(true);

      policies.add(takeoffCoMPolicy);
      policies.add(touchdownCoMPolicy);
      policies.add(touchdownDCMPolicy);

      parentRegistry.addChild(registry);
   }

   public void computeJumpingHeuristics(JumpingGoal jumpingGoal)
   {
      takeOffCoMPosition.setToZero(controllerToolbox.getReferenceFrames().getMidFeetZUpFrame());
      touchdownDCMPosition.setToZero(controllerToolbox.getReferenceFrames().getMidFeetZUpFrame());

      this.touchdownDistance.set(Math.min(computeSymmetricTouchdownDistance(jumpingGoal.getGoalLength(),
                                                                            jumpingGoal.getFlightDuration(),
                                                                            controllerToolbox.getJumpingHeight(),
                                                                            controllerToolbox.getGravityZ()), maximumTouchdownDistance.getDoubleValue()));
      touchdownHeight.set(computeConstrainedHeight(maxLegLength.getDoubleValue(), controllerToolbox.getJumpingHeight(), touchdownDistance.getDoubleValue()));

      takeOffDistance.set(computeTakeOffDistance(jumpingGoal.getGoalLength(),
                                                 touchdownDistance.getDoubleValue(),
                                                 jumpingGoal.getFlightDuration(),
                                                 touchdownHeight.getDoubleValue(),
                                                 controllerToolbox.getGravityZ()));
      takeOffHeight.set(computeConstrainedHeight(maxLegLength.getDoubleValue(), controllerToolbox.getJumpingHeight(), takeOffDistance.getDoubleValue()));
      double jumpHeight = Double.isNaN(jumpingGoal.getGoalHeight()) ? 0.0 : jumpingGoal.getGoalHeight();

      takeOffCoMPosition.addX(takeOffDistance.getDoubleValue());
      takeOffCoMPosition.addZ(takeOffHeight.getDoubleValue());

      touchdownDCMPosition.addX(jumpingGoal.getGoalLength());
      touchdownDCMPosition.addZ(jumpHeight + touchdownHeight.getDoubleValue());

      touchdownCoMPosition.setIncludingFrame(touchdownDCMPosition);
      touchdownCoMPosition.subX(touchdownDistance.getDoubleValue());

      takeOffCoMPosition.changeFrame(worldFrame);
      touchdownDCMPosition.changeFrame(worldFrame);
      touchdownCoMPosition.changeFrame(worldFrame);

      takeoffCoMPolicy.getDesiredComPosition().set(takeOffCoMPosition);
      touchdownCoMPolicy.getDesiredComPosition().set(touchdownCoMPosition);
      touchdownDCMPolicy.getDesiredDCMPosition().set(touchdownDCMPosition);

      takeoffCoMPolicy.setPolicyWeight(desiredWeightForStateChangeHeights.getDoubleValue());
      touchdownCoMPolicy.setPolicyWeight(desiredWeightForStateChangeHeights.getDoubleValue());
      touchdownDCMPolicy.setPolicyWeight(desiredWeightForStateChangeHeights.getDoubleValue());

      takeoffCoMPolicy.setTimeOfPolicy(jumpingGoal.getSupportDuration());
      touchdownCoMPolicy.setTimeOfPolicy(jumpingGoal.getSupportDuration() + jumpingGoal.getFlightDuration());
      touchdownDCMPolicy.setTimeOfPolicy(jumpingGoal.getSupportDuration() + jumpingGoal.getFlightDuration());
   }

   private static double computeSymmetricTouchdownDistance(double jumpLength, double flightDuration, double nominalTouchdownHeight, double gravity)
   {
      double omega = Math.sqrt(gravity / nominalTouchdownHeight);
      return jumpLength / (omega * flightDuration + 2.0);
   }

   public static double computeConstrainedHeight(double maxLegLength, double nominalTouchdownHeight, double touchdownDistance)
   {
      double maxHeight = Math.sqrt(MathTools.square(maxLegLength) - MathTools.square(touchdownDistance));
      return Math.min(nominalTouchdownHeight, maxHeight);
   }

   private static double computeTakeOffDistance(double jumpLength, double touchdownDistance, double flightDuration, double touchdownHeight, double gravity)
   {
      return jumpLength - touchdownDistance * (1.0 + Math.sqrt(gravity / touchdownHeight) * flightDuration);
   }

   public List<CustomMPCPolicy> getCustomPolicies()
   {
      return policies;
   }
}
