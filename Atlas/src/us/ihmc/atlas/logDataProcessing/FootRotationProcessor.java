package us.ihmc.atlas.logDataProcessing;

import us.ihmc.avatar.logProcessor.LogDataProcessorFunction;
import us.ihmc.avatar.logProcessor.LogDataProcessorHelper;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.PartialFootholdControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootRotationProcessor implements LogDataProcessorFunction
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final SideDependentList<PartialFootholdControlModule> partialFootholdControlModules = new SideDependentList<>();
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final LogDataProcessorHelper logDataProcessorHelper;

   public FootRotationProcessor(LogDataProcessorHelper logDataProcessorHelper)
   {
      this.logDataProcessorHelper = logDataProcessorHelper;
      WalkingControllerParameters walkingControllerParameters = logDataProcessorHelper.getWalkingControllerParameters();
      controllerToolbox = logDataProcessorHelper.getHighLevelHumanoidControllerToolbox();

      ExplorationParameters explorationParameters = new ExplorationParameters(registry);
      for (RobotSide robotSide : RobotSide.values)
      {
         PartialFootholdControlModule partialFootholdControlModule = new PartialFootholdControlModule(robotSide, controllerToolbox,
               walkingControllerParameters, explorationParameters, registry, yoGraphicsListRegistry);
         partialFootholdControlModules.put(robotSide, partialFootholdControlModule);
      }
   }

   private final FramePoint2D measuredCoP2d = new FramePoint2D();
   private final FramePoint2D desiredCoP2d = new FramePoint2D();

   @Override
   public void processDataAtControllerRate()
   {
      logDataProcessorHelper.update();

      for (RobotSide robotSide : RobotSide.values)
      {
         PartialFootholdControlModule partialFootholdControlModule = partialFootholdControlModules.get(robotSide);

         if (logDataProcessorHelper.getCurrenFootState(robotSide) == ConstraintType.FULL)
         {
            logDataProcessorHelper.getMeasuredCoP(robotSide, measuredCoP2d);
            logDataProcessorHelper.getDesiredCoP(robotSide, desiredCoP2d);
            partialFootholdControlModule.compute(desiredCoP2d, measuredCoP2d);

            YoPlaneContactState contactState = controllerToolbox.getFootContactStates().get(robotSide);
            boolean contactStateHasChanged = partialFootholdControlModule.applyShrunkPolygon(contactState);
            if (contactStateHasChanged)
            {
               contactState.notifyContactStateHasChanged();
            }
         }
         else
         {
            partialFootholdControlModule.reset();
         }
      }
   }

   @Override
   public void processDataAtStateEstimatorRate()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }
}
