package us.ihmc.valkyrie;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.walkingPreview.WalkingControllerPreviewToolboxController;
import us.ihmc.avatar.networkProcessor.walkingPreview.WalkingControllerPreviewToolboxModule;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

public class ValkyrieWalkingPreviewLauncher
{
   public static void main(String[] args) throws IOException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      WalkingControllerPreviewToolboxModule toolboxModule = new WalkingControllerPreviewToolboxModule(robotModel, false, PubSubImplementation.FAST_RTPS);
      WalkingControllerPreviewToolboxController toolboxController = toolboxModule.getToolboxController();

      Robot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getFullRobotModel());
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoRegistry(toolboxModule.getRegistry());
      scs.addYoGraphicsListRegistry(toolboxModule.getYoGraphicsListRegistry(), true);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(toolboxModule.getYoGraphicsListRegistry());
      simulationOverheadPlotterFactory.createOverheadPlotter();

      toolboxController.addUpdatable(new Updatable()
      {
         @Override
         public void update(double time)
         {
            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
            scs.tickAndUpdate();
         }
      });

      scs.startOnAThread();
   }
}
