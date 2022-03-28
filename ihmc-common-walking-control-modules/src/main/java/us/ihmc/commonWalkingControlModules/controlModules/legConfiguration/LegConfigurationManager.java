package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule.LegControlWeight;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class LegConfigurationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean attemptToStraightenLegs = new YoBoolean("attemptToStraightenLegs", registry);

   private final SideDependentList<LegConfigurationControlModule> legConfigurationControlModules = new SideDependentList<>();

   public LegConfigurationManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                                  YoRegistry parentRegistry)
   {
      LegConfigurationParameters legConfigurationParameters = walkingControllerParameters.getLegConfigurationParameters();
      for (RobotSide robotSide : RobotSide.values)
         legConfigurationControlModules.put(robotSide, new LegConfigurationControlModule(robotSide, controllerToolbox, legConfigurationParameters, registry));

      attemptToStraightenLegs.set(legConfigurationParameters.attemptToStraightenLegs());


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

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return null;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return legConfigurationControlModules.get(robotSide).getInverseDynamicsCommand();
   }
}
