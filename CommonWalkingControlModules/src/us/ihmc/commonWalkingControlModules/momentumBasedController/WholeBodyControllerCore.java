package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreOuput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolderInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.WholeBodyInverseDynamicsSolver;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class WholeBodyControllerCore
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final BooleanYoVariable isEnabled = new BooleanYoVariable("isControllerCoreEnabled", registry);
   private final IntegerYoVariable numberOfFBControllerEnabled = new IntegerYoVariable("numberOfFBControllerEnabled", registry);

   private final WholeBodyFeedbackController feedbackController;
   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;

   private final ControllerCoreOuput controllerCoreOuput;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private OneDoFJoint[] oneDoFJoints;

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         FeedbackControlCommandList allPossibleCommands, YoVariableRegistry parentRegistry)
   {

      feedbackController = new WholeBodyFeedbackController(toolbox, allPossibleCommands, registry);
      inverseDynamicsSolver = new WholeBodyInverseDynamicsSolver(toolbox, momentumOptimizationSettings, registry);
      oneDoFJoints = ScrewTools.filterJoints(inverseDynamicsSolver.getJointsToOptimizeFors(), OneDoFJoint.class);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(oneDoFJoints, registry);

      CenterOfPressureDataHolder desiredCenterOfPressureDataHolder = inverseDynamicsSolver.getDesiredCenterOfPressureDataHolder();
      controllerCoreOuput = new ControllerCoreOuput(desiredCenterOfPressureDataHolder);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      feedbackController.initialize();
      inverseDynamicsSolver.initialize();
      yoLowLevelOneDoFJointDesiredDataHolder.clear();
   }

   public void reset()
   {
      feedbackController.reset();
      inverseDynamicsSolver.reset();
      yoLowLevelOneDoFJointDesiredDataHolder.clear();
   }

   public void submitControllerCoreCommand(ControllerCoreCommand controllerCoreCommand)
   {
      reset();

      isEnabled.set(controllerCoreCommand.enableControllerCore());

      if (isEnabled.getBooleanValue())
      {
         feedbackController.submitFeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());
         inverseDynamicsSolver.submitInverseDynamicsCommand(controllerCoreCommand.getInverseDynamicsCommandList());
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
   }

   public void compute()
   {
      if (isEnabled.getBooleanValue())
      {
         feedbackController.compute();
         InverseDynamicsCommandList feedbackControllerOutput = feedbackController.getOutput();
         numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
         inverseDynamicsSolver.submitInverseDynamicsCommand(feedbackControllerOutput);
         inverseDynamicsSolver.compute();
         LowLevelOneDoFJointDesiredDataHolder solverOutput = inverseDynamicsSolver.getOutput();
         yoLowLevelOneDoFJointDesiredDataHolder.completeWith(solverOutput);
      }
      else
      {
         numberOfFBControllerEnabled.set(0);
         yoLowLevelOneDoFJointDesiredDataHolder.insertDesiredTorquesIntoOneDoFJoints(oneDoFJoints);
      }
   }

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2d desiredCoPToPack)
   {
      inverseDynamicsSolver.getDesiredCenterOfPressure(contactablePlaneBody, desiredCoPToPack);
   }

   public ControllerCoreOuput getOutputForHighLevelController()
   {
      return controllerCoreOuput;
   }

   public LowLevelOneDoFJointDesiredDataHolderInterface getOutputForLowLevelController()
   {
      return yoLowLevelOneDoFJointDesiredDataHolder;
   }
}
