package us.ihmc.commonWalkingControlModules.controlModules.kneeAngle;

import us.ihmc.commonWalkingControlModules.controlModules.foot.KneeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KneeAngleManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<KneeControlModule> kneeControlModules = new SideDependentList<>();

   public KneeAngleManager(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         kneeControlModules.put(robotSide, new KneeControlModule(robotSide, controllerToolbox, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         kneeControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         kneeControlModules.get(robotSide).doControl();
      }
   }

   public void startSwing(RobotSide upcomingSwingSide)
   {
   }

   public void startTransfer(RobotSide upcomingSupportSide)
   {
   }
}
