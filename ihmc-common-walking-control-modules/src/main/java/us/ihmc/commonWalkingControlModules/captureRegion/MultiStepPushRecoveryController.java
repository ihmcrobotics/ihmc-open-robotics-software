package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.function.Function;

public class MultiStepPushRecoveryController
{
   private final YoBoolean useExternalRecoveryFootholds;
   private final MultiStepPushRecoveryModule pushRecoveryModule;
   private final ExternalPushRecoveryStepHandler externalPushRecoveryStepHandler;

   private final SideDependentList<YoPlaneContactState> contactStates;

   public MultiStepPushRecoveryController(SideDependentList<YoPlaneContactState> contactStates,
                                          BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                          FrameConvexPolygon2DReadOnly defaultSupportPolygon,
                                          PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                          YoRegistry parentRegistry,
                                          YoGraphicsListRegistry graphicsListRegistry)
   {
      this.contactStates = contactStates;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      externalPushRecoveryStepHandler = new ExternalPushRecoveryStepHandler(pushRecoveryControllerParameters, registry);
      useExternalRecoveryFootholds = new YoBoolean("useExternalRecoveryFootholds", parentRegistry);
      this.pushRecoveryModule = new MultiStepPushRecoveryModule(new IsInContactFunction(),
                                                                bipedSupportPolygons.getSupportPolygonInWorld(),
                                                                bipedSupportPolygons.getFootPolygonsInWorldFrame(),
                                                                soleZUpFrames,
                                                                defaultSupportPolygon,
                                                                pushRecoveryControllerParameters,
                                                                registry,
                                                                graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      pushRecoveryModule.reset();
      externalPushRecoveryStepHandler.reset();
   }

   public boolean isRobotFallingFromDoubleSupport()
   {
      return pushRecoveryModule.isRobotFallingFromDoubleSupport();
   }

   public RobotSide getSwingSideForRecovery()
   {
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.getSwingSideForRecovery();
      else
         return pushRecoveryModule.getSwingSideForRecovery();
   }

   public int getNumberOfRecoverySteps()
   {
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.getNumberOfRecoverySteps();
      else
         return pushRecoveryModule.getNumberOfRecoverySteps();
   }

   public Footstep pollRecoveryStep()
   {
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.pollRecoveryStep();
      else
         return pushRecoveryModule.pollRecoveryStep();
   }

   public FootstepTiming pollRecoveryStepTiming()
   {
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.pollRecoveryStepTiming();
      else
         return pushRecoveryModule.pollRecoveryStepTiming();
   }

   public Footstep getRecoveryStep(int stepIndex)
   {
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.getRecoveryStep(stepIndex);
      else
         return pushRecoveryModule.getRecoveryStep(stepIndex);
   }

   public FootstepTiming getRecoveryStepTiming(int stepIndex)
   {
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.getRecoveryStepTiming(stepIndex);
      else
         return pushRecoveryModule.getRecoveryStepTiming(stepIndex);
   }

   public boolean isRecoveryImpossible()
   {
      if (pushRecoveryModule.isRecoveryImpossible())
         return true;
      if (useExternalRecoveryFootholds.getValue())
         return externalPushRecoveryStepHandler.isRecoveryImpossible();

      return false;
   }

   public void updateForDoubleSupport(FramePoint2DReadOnly capturePoint2d, double omega0)
   {
      pushRecoveryModule.updateForDoubleSupport(capturePoint2d, omega0);
   }

   private class IsInContactFunction implements Function<RobotSide, Boolean>
   {
      public Boolean apply(RobotSide robotSide)
      {
         if (contactStates.get(robotSide).inContact())
            return Boolean.TRUE;
         else
            return Boolean.FALSE;
      }
   }
}
