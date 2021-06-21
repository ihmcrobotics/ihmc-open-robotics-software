package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.function.Function;

public class MultiStepPushRecoveryController
{
   private final MultiStepPushRecoveryModule pushRecoveryModule;

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
      this.pushRecoveryModule = new MultiStepPushRecoveryModule(new IsInContactFunction(),
                                                                bipedSupportPolygons.getSupportPolygonInWorld(),
                                                                bipedSupportPolygons.getFootPolygonsInWorldFrame(),
                                                                soleZUpFrames,
                                                                defaultSupportPolygon,
                                                                pushRecoveryControllerParameters,
                                                                parentRegistry,
                                                                graphicsListRegistry);
   }

   public void reset()
   {
      pushRecoveryModule.reset();
   }

   public boolean isRobotFallingFromDoubleSupport()
   {
      return pushRecoveryModule.isRobotFallingFromDoubleSupport();
   }

   public RobotSide getSwingSideForRecovery()
   {
      return pushRecoveryModule.getSwingSideForRecovery();
   }

   public int getNumberOfRecoverySteps()
   {
      return pushRecoveryModule.getNumberOfRecoverySteps();
   }

   public Footstep pollRecoveryStep()
   {
      return pushRecoveryModule.pollRecoveryStep();
   }

   public FootstepTiming pollRecoveryStepTiming()
   {
      return pushRecoveryModule.pollRecoveryStepTiming();
   }

   public Footstep getRecoveryStep(int stepIndex)
   {
      return pushRecoveryModule.getRecoveryStep(stepIndex);
   }

   public FootstepTiming getRecoveryStepTiming(int stepIndex)
   {
      return pushRecoveryModule.getRecoveryStepTiming(stepIndex);
   }

   public boolean isRecoveryImpossible()
   {
      return pushRecoveryModule.isRecoveryImpossible();
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
