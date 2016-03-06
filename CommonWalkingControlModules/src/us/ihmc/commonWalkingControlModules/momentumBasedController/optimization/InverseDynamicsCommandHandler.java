package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;

import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class InverseDynamicsCommandHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CentroidalMomentumHandler centroidalMomentumHandler;
   private final PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator;

   private final InverseDynamicsMotionQPInput motionQPInput;

   public InverseDynamicsCommandHandler(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, GeometricJacobianHolder geometricJacobianHolder,
         int numberOfDoFs, int rhoSize, int maxNPointsPerPlane, int maxNSupportVectors, double wRho, double wRhoSmoother, double wRhoPenalizer,
         List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, registry);
      wrenchMatrixCalculator = new PlaneContactWrenchMatrixCalculator(centerOfMassFrame, rhoSize, maxNPointsPerPlane, maxNSupportVectors, wRho, wRhoSmoother,
            wRhoPenalizer, contactablePlaneBodies, registry);
      motionQPInput = new InverseDynamicsMotionQPInput(numberOfDoFs);
   }

   public InverseDynamicsMotionQPInput computeMotionTask(SpatialAccelerationCommand command)

   {
      motionQPInput.setIsMotionConstraint(command.getHasWeight());
      if (command.getHasWeight())
      {
         motionQPInput.setUseWeightScalar(true);
         motionQPInput.setWeight(command.getWeight());
      }

      RigidBody base = command.getBase();
      RigidBody endEffector = command.getEndEffector();
 
      return motionQPInput;
   }
}
