package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointAccelerationHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointTorqueHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class WholeBodyInverseDynamicsControlCore
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final OptimizationMomentumControlModule optimizationMomentumControlModule;

   private final DesiredOneDoFJointTorqueHolder desiredOneDoFJointTorqueHolder;
   private final DesiredOneDoFJointAccelerationHolder desiredOneDoFJointAccelerationHolder;

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final InverseDynamicsJoint[] jointsToOptimizeFors;

   public WholeBodyInverseDynamicsControlCore(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         YoVariableRegistry parentRegistry)
   {
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double gravityZ = toolbox.getGravityZ();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();
      YoGraphicsListRegistry yoGraphicsListRegistry = toolbox.getYoGraphicsListRegistry();

      inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      optimizationMomentumControlModule = new OptimizationMomentumControlModule(toolbox, momentumOptimizationSettings, registry);

      jointsToOptimizeFors = momentumOptimizationSettings.getJointsToOptimizeFor();
      desiredOneDoFJointTorqueHolder = new DesiredOneDoFJointTorqueHolder(jointsToOptimizeFors, registry);
      desiredOneDoFJointAccelerationHolder = new DesiredOneDoFJointAccelerationHolder(jointsToOptimizeFors, registry);

      planeContactWrenchProcessor = new PlaneContactWrenchProcessor(contactablePlaneBodies, yoGraphicsListRegistry, registry);

      wrenchVisualizer = WrenchVisualizer.createWrenchVisualizerWithContactableBodies("DesiredExternalWrench", contactablePlaneBodies, 1.0,
            yoGraphicsListRegistry, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      inverseDynamicsCalculator.reset();
      optimizationMomentumControlModule.reset();
      desiredOneDoFJointTorqueHolder.reset();
      desiredOneDoFJointAccelerationHolder.reset();
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      inverseDynamicsCalculator.compute();
      optimizationMomentumControlModule.initialize();
      planeContactWrenchProcessor.initialize();
   }

   public void compute()
   {
      MomentumModuleSolution momentumModuleSolution;

      try
      {
         momentumModuleSolution = optimizationMomentumControlModule.compute();
      }
      catch (MomentumControlModuleException momentumControlModuleException)
      {
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
      }

      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
      List<RigidBody> rigidBodiesWithExternalWrench = momentumModuleSolution.getRigidBodiesWithExternalWrench();

      for (int i = 0; i < rigidBodiesWithExternalWrench.size(); i++)
      {
         RigidBody rigidBody = rigidBodiesWithExternalWrench.get(i);
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenchSolution.get(rigidBody));
      }

      inverseDynamicsCalculator.compute();
      desiredOneDoFJointTorqueHolder.extractDesiredTorquesFromInverseDynamicsJoints(jointsToOptimizeFors);
      desiredOneDoFJointAccelerationHolder.extractDesiredAccelerationsFromInverseDynamicsJoints(jointsToOptimizeFors);
      planeContactWrenchProcessor.compute(externalWrenchSolution);
      wrenchVisualizer.visualize(externalWrenchSolution);
   }

   public void submitInverseDynamicsCommand(InverseDynamicsCommand<?> inverseDynamicsCommand)
   {
      optimizationMomentumControlModule.setInverseDynamicsCommand(inverseDynamicsCommand);
   }

   public DesiredOneDoFJointTorqueHolder getDesiredOneDoFJointTorqueHolder()
   {
      return desiredOneDoFJointTorqueHolder;
   }

   public DesiredOneDoFJointAccelerationHolder getDesiredOneDoFJointAccelerationHolder()
   {
      return desiredOneDoFJointAccelerationHolder;
   }

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2d desiredCoPToPack)
   {
      planeContactWrenchProcessor.getDesiredCenterOfPressure(contactablePlaneBody, desiredCoPToPack);
   }
}
