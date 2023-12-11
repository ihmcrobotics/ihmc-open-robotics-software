package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WholeBodyControllerCoreFactory
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private LinearMomentumRateControlModule linearMomentumRateControlModule;
   private WholeBodyControllerCore controllerCore;
   private WholeBodyControlCoreToolbox toolbox;

   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private WalkingControllerParameters walkingControllerParameters;
   private final FeedbackControllerTemplate template = new FeedbackControllerTemplate();

   public WholeBodyControllerCoreFactory(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   private boolean hasHighLevelHumanoidControllerToolbox(Class<?> managerClass)
   {
      if (controllerToolbox != null)
         return true;
      missingObjectWarning(HighLevelHumanoidControllerToolbox.class, managerClass);
      return false;
   }

   private boolean hasWalkingControllerParameters(Class<?> managerClass)
   {
      if (walkingControllerParameters != null)
         return true;
      missingObjectWarning(WalkingControllerParameters.class, managerClass);
      return false;
   }

   private void missingObjectWarning(Class<?> missingObjectClass, Class<?> managerClass)
   {
      LogTools.warn(missingObjectClass.getSimpleName() + " has not been set, cannot create: " + managerClass.getSimpleName());
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }

   public void setWalkingControllerParameters(WalkingControllerParameters walkingControllerParameters)
   {
      this.walkingControllerParameters = walkingControllerParameters;
   }

   public void setFeedbackControllerTemplate(FeedbackControllerTemplate template)
   {
      if (controllerCore == null)
         this.template.mergeWithOtherTemplate(template);
      else
         controllerCore.registerAdditionalControllers(template);
   }

   public WholeBodyControlCoreToolbox getOrCreateWholeBodyControllerCoreToolbox()
   {
      if (toolbox != null)
         return toolbox;

      if (!hasHighLevelHumanoidControllerToolbox(WholeBodyControllerCore.class))
         return null;
      if (!hasWalkingControllerParameters(WholeBodyControllerCore.class))
         return null;

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      JointBasics[] jointsToOptimizeFor = controllerToolbox.getControlledJoints();

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      toolbox = new WholeBodyControlCoreToolbox(controllerToolbox.getControlDT(),
                                                controllerToolbox.getGravityZ(),
                                                rootJoint,
                                                jointsToOptimizeFor,
                                                centerOfMassFrame,
                                                walkingControllerParameters.getMomentumOptimizationSettings(),
                                                controllerToolbox.getYoGraphicsListRegistry(),
                                                registry);
      return toolbox;
   }

   public WholeBodyControllerCore getWholeBodyControllerCore()
   {
      return controllerCore;
   }

   public WholeBodyControllerCore getOrCreateWholeBodyControllerCore()
   {
      if (controllerCore != null)
         return controllerCore;

      if (!hasHighLevelHumanoidControllerToolbox(WholeBodyControllerCore.class))
         return null;
      if (!hasWalkingControllerParameters(WholeBodyControllerCore.class))
         return null;

      WholeBodyControlCoreToolbox toolbox = getOrCreateWholeBodyControllerCoreToolbox();

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      toolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      toolbox.setFeedbackControllerSettings(walkingControllerParameters.getFeedbackControllerSettings());

      String[] inactiveJoints = walkingControllerParameters.getInactiveJoints();
      if (inactiveJoints != null)
      {
         for (String inactiveJoint : inactiveJoints)
            toolbox.addInactiveJoint(fullRobotModel.getOneDoFJointByName(inactiveJoint));
      }

      toolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());
      fullRobotModel.getKinematicLoops().forEach(toolbox::addKinematicLoopFunction);
      // IMPORTANT: Cannot allow dynamic construction in a real-time environment such as this controller. This needs to be false.
      template.setAllowDynamicControllerConstruction(false);
      OneDoFJointBasics[] controlledJoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(controlledJoints);
      controllerCore = new WholeBodyControllerCore(toolbox, template, lowLevelControllerOutput, registry);

      return controllerCore;
   }

   public LinearMomentumRateControlModule getLinearMomentumRateControlModule()
   {
      return linearMomentumRateControlModule;
   }

   public LinearMomentumRateControlModule getOrCreateLinearMomentumRateControlModule(YoRegistry registry)
   {
      if (linearMomentumRateControlModule != null)
         return linearMomentumRateControlModule;

      if (!hasHighLevelHumanoidControllerToolbox(LinearMomentumRateControlModule.class))
         return null;
      if (!hasWalkingControllerParameters(LinearMomentumRateControlModule.class))
         return null;

      CenterOfMassStateProvider centerOfMassStateProvider = controllerToolbox;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      double controlDT = controllerToolbox.getControlDT();
      double gravityZ = controllerToolbox.getGravityZ();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();

      linearMomentumRateControlModule = new LinearMomentumRateControlModule(centerOfMassStateProvider,
                                                                            referenceFrames,
                                                                            contactableFeet,
                                                                            elevator,
                                                                            walkingControllerParameters,
                                                                            gravityZ,
                                                                            controlDT,
                                                                            registry,
                                                                            yoGraphicsListRegistry);

      return linearMomentumRateControlModule;
   }

}
