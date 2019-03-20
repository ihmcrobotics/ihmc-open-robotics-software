package us.ihmc.exampleSimulations.beetle.controller;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.exampleSimulations.beetle.footContact.SimulatedPlaneContactStateUpdater;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class HexapodHighLevelControlManager
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final ControllerCoreCommand controllerCoreCommandList;
   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode = new YoEnum<>("controllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final HexapodBodySpatialManager bodySpatialManager;
   private final HexapodMomentumController hexapodMomentumController;
   private final HexapodStepController stepController;

   private HexapodControllerParameters currentParameters;
   private final HexapodControllerParameters vmcParameters;
   private final HexapodControllerParameters idParameters;

   public HexapodHighLevelControlManager(FullRobotModel fullRobotModel, HexapodReferenceFrames referenceFrames, SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters,
         ArrayList<String> jointsToControl, HexapodControllerParameters idParameters, HexapodControllerParameters vmcParameters, YoGraphicsListRegistry yoGraphicsListRegistry, double controllerDt, YoVariableRegistry parentRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.idParameters = idParameters;
      this.vmcParameters = vmcParameters;

      controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      controllerCoreCommandList = new ControllerCoreCommand(controllerCoreMode.getEnumValue());

      RigidBodyBasics body = fullRobotModel.getRootBody();
      String bodyName = body.getName();

      bodySpatialManager = new HexapodBodySpatialManager(bodyName, fullRobotModel, referenceFrames, controllerDt, yoGraphicsListRegistry, registry);
      stepController = new HexapodStepController("HexapodStepController", fullRobotModel, contactStateUpdaters, yoGraphicsListRegistry, controllerDt, registry, referenceFrames);
      hexapodMomentumController = new HexapodMomentumController("HexapodMomentumController", referenceFrames, fullRobotModel, controllerDt, registry, yoGraphicsListRegistry);
      
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      bodySpatialManager.initialize();
      stepController.initialize();
      hexapodMomentumController.initialize();
   }

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   public void doControl()
   {
      controllerCoreCommandList.clear();
      WholeBodyControllerCoreMode controlMode = controllerCoreMode.getEnumValue();
      controllerCoreCommandList.setControllerCoreMode(controlMode);
      updateCurrentParameters(controlMode);

      bodySpatialManager.doControl(currentParameters);
      bodySpatialManager.getDesiredLinearVelocity(linearVelocity);
      bodySpatialManager.getDesiredAngularVelocity(angularVelocity);
      stepController.doControl(currentParameters, linearVelocity, angularVelocity);
      hexapodMomentumController.doControl();
      
      controllerCoreCommandList.addVirtualModelControlCommand(hexapodMomentumController.getMomentumRateCommand());
      controllerCoreCommandList.addVirtualModelControlCommand(hexapodMomentumController.getMomentumRateCommand());
      controllerCoreCommandList.addVirtualModelControlCommand(stepController.getVirtualModelControlCommand());
      controllerCoreCommandList.addInverseDynamicsCommand(stepController.getInverseDynamicsCommand());
      controllerCoreCommandList.addFeedbackControlCommand(bodySpatialManager.getSpatialFeedbackControlCommand());
      controllerCoreCommandList.addFeedbackControlCommand(stepController.getFeedbackCommandList());
   }

   private void updateCurrentParameters(WholeBodyControllerCoreMode controlMode)
   {
      if(controlMode == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
      {
         currentParameters = idParameters;
      }
      else
      {
         currentParameters = vmcParameters;
      }
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommandList;
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      controllerCoreCommandList.clear();
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      
      ret.addCommand(bodySpatialManager.getSpatialFeedbackControlCommand());
      ret.addCommand(stepController.getFeedbackControlTemplate());

      return ret;
   }
}
