package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class InverseDynamicsOptimizationControlModule
{
   private static final int nBasisVectorsPerContactPoint = 4;
   private static final int nContactPointsPerContactableBody = 4;
   private static final int nContactableBody = 4;
   private static final int rhoSize = nContactableBody * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseDynamicsJoint[] jointsToOptimizeFor;

   private final InverseDynamicsQPSolver qpSolver;

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings, YoVariableRegistry parentRegistry)
   {
      this.jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();
      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      qpSolver = new InverseDynamicsQPSolver(numberOfDoFs, rhoSize, registry);

      parentRegistry.addChild(registry);
   }
}
