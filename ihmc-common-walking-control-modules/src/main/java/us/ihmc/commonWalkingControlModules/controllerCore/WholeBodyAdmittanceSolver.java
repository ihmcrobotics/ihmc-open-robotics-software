package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.JointAdmittanceConfigurationHandler;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WholeBodyAdmittanceSolver
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;
   private final JointAdmittanceConfigurationHandler admittanceConfigurationHandler;

   public WholeBodyAdmittanceSolver(WholeBodyControlCoreToolbox toolbox, YoRegistry parentRegistry)
   {
      inverseDynamicsSolver = new WholeBodyInverseDynamicsSolver(toolbox, registry);
      admittanceConfigurationHandler = new JointAdmittanceConfigurationHandler(toolbox.getJointIndexHandler().getIndexedOneDoFJoints(), registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      inverseDynamicsSolver.reset();
   }

   public void initialize()
   {
      inverseDynamicsSolver.initialize();
   }

   public void compute()
   {
      inverseDynamicsSolver.compute();
      admittanceConfigurationHandler.computeJointAdmittanceSetpoints(inverseDynamicsSolver.getOutput());
   }

   public void submitInverseDynamicsCommandList(InverseDynamicsCommandList inverseDynamicsCommandList)
   {
      inverseDynamicsSolver.submitInverseDynamicsCommandList(inverseDynamicsCommandList);
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return admittanceConfigurationHandler.getOutput();
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return inverseDynamicsSolver.getOutputForRootJoint();
   }

   public FrameVector3DReadOnly getAchievedMomentumRateLinear()
   {
      return inverseDynamicsSolver.getAchievedMomentumRateLinear();
   }

   public FrameVector3DReadOnly getAchievedMomentumRateAngular()
   {
      return inverseDynamicsSolver.getAchievedMomentumRateAngular();
   }
}
