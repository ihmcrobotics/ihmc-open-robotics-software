package us.ihmc.exampleSimulations.beetle.controller;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.ControlledBodiesCommand;
import us.ihmc.exampleSimulations.beetle.footContact.SimulatedPlaneContactStateUpdater;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class HexapodHighLevelControlManager
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final ControlledBodiesCommand controlledBodiesCommand = new ControlledBodiesCommand();
   private final ControllerCoreCommand controllerCoreCommandList;
   private final EnumYoVariable<WholeBodyControllerCoreMode> controllerCoreMode = new EnumYoVariable<>("controllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final BodySpatialManager bodySpatialManager;
   private final ArrayList<HighLevelJointManager> jointControllers = new ArrayList<>();
   private final HexapodMomentumController hexapodMomentumController;
   private final HexapodStepController stepController;

   private HexapodControllerParameters currentParameters;
   private final HexapodControllerParameters vmcParameters;
   private final HexapodControllerParameters idParameters;

   public HexapodHighLevelControlManager(FullRobotModel fullRobotModel, HexapodReferenceFrames referenceFrames, TwistCalculator twistCalculator, SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters,
         ArrayList<String> jointsToControl, HexapodControllerParameters idParameters, HexapodControllerParameters vmcParameters, YoGraphicsListRegistry yoGraphicsListRegistry, double controllerDt, YoVariableRegistry parentRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.idParameters = idParameters;
      this.vmcParameters = vmcParameters;

      controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      controllerCoreCommandList = new ControllerCoreCommand(controllerCoreMode.getEnumValue());

      RigidBody pelvis = fullRobotModel.getPelvis();
      String bodyName = pelvis.getName();

      bodySpatialManager = new BodySpatialManager(bodyName, fullRobotModel, referenceFrames, controllerDt, yoGraphicsListRegistry, registry);
      stepController = new HexapodStepController("HexapodStepController", fullRobotModel, twistCalculator, contactStateUpdaters, yoGraphicsListRegistry, controllerDt, registry, referenceFrames);
      hexapodMomentumController = new HexapodMomentumController("HexapodMomentumController", referenceFrames, fullRobotModel, controllerDt, registry, yoGraphicsListRegistry);
      
      if(jointsToControl != null)
      {
         for(String jointName : jointsToControl)
         {
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            HighLevelJointManager jointManager = new HighLevelJointManager(jointName, joint, registry);
            jointControllers.add(jointManager);
         }
      }
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      bodySpatialManager.initialize();
      stepController.initialize();
      hexapodMomentumController.initialize();
      for(HighLevelJointManager jointManager : jointControllers)
      {
         jointManager.initialize();
      }
   }

   private final FrameVector linearVelocity = new FrameVector();
   private final FrameVector angularVelocity = new FrameVector();
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
      
      for(HighLevelJointManager jointManager : jointControllers)
      {
         jointManager.doControl(currentParameters);
         controllerCoreCommandList.addFeedbackControlCommand(jointManager.getJointspaceFeedbackControlCommand());
      }

      controllerCoreCommandList.addVirtualModelControlCommand(hexapodMomentumController.getMomentumRateCommand());
      controllerCoreCommandList.addVirtualModelControlCommand(hexapodMomentumController.getMomentumRateCommand());
      controllerCoreCommandList.addVirtualModelControlCommand(stepController.getContactStates());
      controllerCoreCommandList.addInverseDynamicsCommand(stepController.getContactStates());
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
      
//      for(HighLevelJointManager jointManager : jointControllers)
//      {
//         addBodiesToControl(jointManager.getRigidBodiesToControl());
//         ret.addCommand(jointManager.getJointspaceFeedbackControlCommand());
//      }
      
      addBodiesToControl(bodySpatialManager.getRigidBodiesToControl());
      addBodiesToControl(stepController.getRigidBodiesToControl());
      controllerCoreCommandList.addVirtualModelControlCommand(controlledBodiesCommand);

      ret.addCommand(bodySpatialManager.getSpatialFeedbackControlCommand());
      ret.addCommand(stepController.getFeedbackControlTemplate());

      return ret;
   }
   
   private void addBodiesToControl(RigidBody[] rigidBodiesToControl)
   {
      for (RigidBody rigidBody : rigidBodiesToControl)
      {
         controlledBodiesCommand.addBodyToControl(rigidBody);
      }
   }
}
