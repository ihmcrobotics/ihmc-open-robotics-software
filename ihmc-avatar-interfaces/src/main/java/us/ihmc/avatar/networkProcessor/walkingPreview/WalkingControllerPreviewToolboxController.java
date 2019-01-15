package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingControllerPreviewToolboxController extends ToolboxController
{
   private final double gravityZ = 9.81;
   private final YoDouble previewTime;
   private final double integrationDT = 0.001;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   // TODO Think how the foot switch should be updated/implemented. We can use the parameter WalkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone to finish swing but is that enough?
   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final WalkingHighLevelHumanoidController walkingController;

   public WalkingControllerPreviewToolboxController(DRCRobotModel robotModel, CommandInputManager commandInputManager,
                                                    StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                    YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      FeedbackControllerSettings feedbackControllerSettings = walkingControllerParameters.getFeedbackControllerSettings();

      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      previewTime = new YoDouble("timeInPreview", registry);
      double omega0 = walkingControllerParameters.getOmega0();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionaContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionaContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();

      List<ContactablePlaneBody> allContactableBodies = new ArrayList<>(additionalContacts);
      allContactableBodies.addAll(feet.values());

      footSwitches = new SideDependentList<>();
      double robotMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.put(robotSide, new SettableFootSwitch(feet.get(robotSide), robotMass, 2, registry));
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, null, null, previewTime, gravityZ, omega0, feet,
                                                                 integrationDT, Collections.emptyList(), allContactableBodies, yoGraphicsListRegistry,
                                                                 jointsToIgnore);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      HighLevelControlManagerFactory managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 controllerToolbox);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      JointBasics[] controlledJoints = controllerToolbox.getControlledJoints();
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(integrationDT, robotMass, fullRobotModel.getRootJoint(),
                                                                                       controlledJoints, controllerToolbox.getCenterOfMassFrame(),
                                                                                       momentumOptimizationSettings, yoGraphicsListRegistry, registry);
      controlCoreToolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      controlCoreToolbox.setFeedbackControllerSettings(feedbackControllerSettings);
      controlCoreToolbox.setupForInverseDynamicsSolver(allContactableBodies);

      FeedbackControlCommandList feedbackControlTemplate = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(controllerToolbox.getControlledOneDoFJoints());
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackControlTemplate, jointDesiredOutputList, registry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());
      registry.addChild(walkingController.getYoVariableRegistry());
   }

   @Override
   public boolean initialize()
   {
      return false;
   }

   @Override
   public void updateInternal() throws Exception
   {
      previewTime.add(integrationDT);
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
