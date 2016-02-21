package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreOuput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointAccelerationHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointTorqueHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.WholeBodyInverseDynamicsSolver;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;

public class WholeBodyControllerCore
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WholeBodyFeedbackController feedbackController;
   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;

   private final ControllerCoreOuput controllerCoreOuput;

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         FeedbackControlCommandList allPossibleCommands, YoVariableRegistry parentRegistry)
   {

      feedbackController = new WholeBodyFeedbackController(toolbox, allPossibleCommands, registry);
      inverseDynamicsSolver = new WholeBodyInverseDynamicsSolver(toolbox, momentumOptimizationSettings, registry);

      CenterOfPressureDataHolder desiredCenterOfPressureDataHolder = inverseDynamicsSolver.getDesiredCenterOfPressureDataHolder();
      controllerCoreOuput = new ControllerCoreOuput(desiredCenterOfPressureDataHolder);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      feedbackController.initialize();
      inverseDynamicsSolver.initialize();
   }

   public void submitControllerCoreCommandList(ControllerCoreCommandList controllerCoreCommandList)
   {
      feedbackController.reset();
      inverseDynamicsSolver.reset();

      feedbackController.submitFeedbackControlCommandList(controllerCoreCommandList.getFeedbackControlCommandList());
      inverseDynamicsSolver.submitInverseDynamicsCommand(controllerCoreCommandList.getInverseDynamicsCommandList());
   }

   public void compute()
   {
      feedbackController.compute();
      inverseDynamicsSolver.submitInverseDynamicsCommand(feedbackController.getOutput());
      inverseDynamicsSolver.compute();
   }

   public DesiredOneDoFJointTorqueHolder getDesiredOneDoFJointTorqueHolder()
   {
      return inverseDynamicsSolver.getDesiredOneDoFJointTorqueHolder();
   }

   public DesiredOneDoFJointAccelerationHolder getDesiredOneDoFJointAccelerationHolder()
   {
      return inverseDynamicsSolver.getDesiredOneDoFJointAccelerationHolder();
   }

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2d desiredCoPToPack)
   {
      inverseDynamicsSolver.getDesiredCenterOfPressure(contactablePlaneBody, desiredCoPToPack);
   }

   public ControllerCoreOuput getOutputForHighLevelController()
   {
      return controllerCoreOuput;
   }
}
