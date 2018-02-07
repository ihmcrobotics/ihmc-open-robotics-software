package us.ihmc.atlas.FlightPhaseTests;

import org.junit.Before;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FlightPhaseControllerCoreTest
{
   private YoVariableRegistry registry;
   private YoGraphicsListRegistry graphicsListRegistry;
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControlCoreToolbox controllerCoreToolbox;
   private WholeBodyControllerCore controllerCore;
   private HumanoidReferenceFrames referenceFrames;
   private WalkingControllerParameters walkingControllerParameters;
   private HighLevelControlManagerFactory managerFactory;
   private final double gravityZ = 9.81;

   @Before
   private void setupTest()
   {
      registry = new YoVariableRegistry(getClass().getSimpleName() + "Registry");
      graphicsListRegistry = new YoGraphicsListRegistry();
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      walkingControllerParameters = robotModel.getWalkingControllerParameters();
      controllerCoreToolbox = new WholeBodyControlCoreToolbox(robotModel.getControllerDT(), gravityZ, fullRobotModel.getRootJoint(),
                                                              fullRobotModel.getControllableOneDoFJoints(), referenceFrames.getCenterOfMassFrame(),
                                                              walkingControllerParameters.getMomentumOptimizationSettings(), graphicsListRegistry, registry);
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
      managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
      FeedbackControlCommandList feedbackCommandTemplate = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList controllerOutputHolder = new JointDesiredOutputList(joints);
      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, feedbackCommandTemplate, controllerOutputHolder, registry);
   }

}