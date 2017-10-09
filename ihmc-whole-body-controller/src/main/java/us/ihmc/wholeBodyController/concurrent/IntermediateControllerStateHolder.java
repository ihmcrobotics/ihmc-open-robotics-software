package us.ihmc.wholeBodyController.concurrent;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.screwTheory.InverseDynamicsJointDesiredAccelerationCopier;

public class IntermediateControllerStateHolder
{
   private final InverseDynamicsJointDesiredAccelerationCopier controllerToIntermediateCopier;
   private final InverseDynamicsJointDesiredAccelerationCopier intermediateToEstimatorCopier;


   public IntermediateControllerStateHolder(FullRobotModelFactory robotModelFactory, FullRobotModel estimatorModel,
         FullRobotModel controllerModel)
   {
      FullRobotModel intermediateModel = robotModelFactory.createFullRobotModel();

      controllerToIntermediateCopier = new InverseDynamicsJointDesiredAccelerationCopier(controllerModel.getElevator(), intermediateModel.getElevator());
      intermediateToEstimatorCopier = new InverseDynamicsJointDesiredAccelerationCopier(intermediateModel.getElevator(), estimatorModel.getElevator());

   }

   public void setFromController()
   {
      controllerToIntermediateCopier.copy();
   }

   public void getIntoEstimator()
   {
      intermediateToEstimatorCopier.copy();
   }

   public static class Builder implements us.ihmc.concurrent.Builder<IntermediateControllerStateHolder>
   {

      private final FullRobotModel estimatorModel;
      private final FullRobotModel controllerModel;
      private final FullRobotModelFactory robotModelFactory;

      public Builder(FullRobotModelFactory robotModelFactory, FullRobotModel estimatorModel, FullRobotModel controllerModel)
      {
         this.robotModelFactory = robotModelFactory;
         this.estimatorModel = estimatorModel;
         this.controllerModel = controllerModel;
      }

      @Override
      public IntermediateControllerStateHolder newInstance()
      {
         return new IntermediateControllerStateHolder(robotModelFactory, estimatorModel, controllerModel);
      }

   }
}
