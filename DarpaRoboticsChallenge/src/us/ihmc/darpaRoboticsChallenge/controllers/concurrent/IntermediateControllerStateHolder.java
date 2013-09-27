package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.darpaRoboticsChallenge.controllers.EstimationLinkHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.utilities.GenericCRC32;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointDesiredAccelerationChecksum;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointDesiredAccelerationCopier;

public class IntermediateControllerStateHolder
{
   private final GenericCRC32 estimatorChecksumCalculator = new GenericCRC32();
   private final GenericCRC32 controllerChecksumCalculator = new GenericCRC32();
   private long checksum;
   
   private final InverseDynamicsJointDesiredAccelerationCopier controllerToIntermediateCopier;
   private final InverseDynamicsJointDesiredAccelerationCopier intermediateToEstimatorCopier;

   private final InverseDynamicsJointDesiredAccelerationChecksum controllerChecksum;
   private final InverseDynamicsJointDesiredAccelerationChecksum estimatorChecksum;
   
   
   private final StateEstimationDataFromController stateEstimationDataFromControllerSink;
   private final StateEstimationDataFromController stateEstimationDataFromControllerIntermediate;
   private final StateEstimationDataFromController stateEstimationDataFromControllerSource;

   public IntermediateControllerStateHolder(SDFFullRobotModelFactory fullRobotModelFactory, SDFFullRobotModel estimatorModel,
         SDFFullRobotModel controllerModel, StateEstimationDataFromController stateEstimationDataFromControllerSink,
         StateEstimationDataFromController stateEstimationDataFromControllerSource)
   {
      SDFFullRobotModel intermediateModel = fullRobotModelFactory.create();

      controllerToIntermediateCopier = new InverseDynamicsJointDesiredAccelerationCopier(controllerModel.getElevator(), intermediateModel.getElevator());
      intermediateToEstimatorCopier = new InverseDynamicsJointDesiredAccelerationCopier(intermediateModel.getElevator(), estimatorModel.getElevator());
      
      controllerChecksum = new InverseDynamicsJointDesiredAccelerationChecksum(controllerModel.getElevator(), controllerChecksumCalculator);
      estimatorChecksum = new InverseDynamicsJointDesiredAccelerationChecksum(estimatorModel.getElevator(), estimatorChecksumCalculator);

      this.stateEstimationDataFromControllerSink = stateEstimationDataFromControllerSink;
      this.stateEstimationDataFromControllerIntermediate = new StateEstimationDataFromController(EstimationLinkHolder.getEstimationFrame(intermediateModel));
      this.stateEstimationDataFromControllerSource = stateEstimationDataFromControllerSource;
   }

   public void setFromController()
   {
      checksum = calculateControllerChecksum();
      controllerToIntermediateCopier.copy();
      stateEstimationDataFromControllerIntermediate.set(stateEstimationDataFromControllerSink);
   }

   public void getIntoEstimator()
   {
      intermediateToEstimatorCopier.copy();
      stateEstimationDataFromControllerSource.set(stateEstimationDataFromControllerIntermediate);
   }
   
   private long calculateControllerChecksum()
   {
      controllerChecksumCalculator.reset();
      controllerChecksum.calculate();
      stateEstimationDataFromControllerSink.calculateChecksum(controllerChecksumCalculator);
      return controllerChecksumCalculator.getValue();
   }
   
   private long calculateEstimatorChecksum()
   {
      estimatorChecksumCalculator.reset();
      estimatorChecksum.calculate();
      stateEstimationDataFromControllerSource.calculateChecksum(estimatorChecksumCalculator);
      return estimatorChecksumCalculator.getValue();
   }
   
   public void validate()
   {
      if(checksum != calculateEstimatorChecksum())
      {
         throw new RuntimeException("Checksum doesn't match expected");
      }
   }

   public static class Builder implements us.ihmc.concurrent.Builder<IntermediateControllerStateHolder>
   {

      private final SDFFullRobotModelFactory fullRobotModelFactory;
      private final SDFFullRobotModel estimatorModel;
      private final SDFFullRobotModel controllerModel;

      private final StateEstimationDataFromController stateEstimationDataFromControllerSink;
      private final StateEstimationDataFromController stateEstimationDataFromControllerSource;

      public Builder(SDFFullRobotModelFactory fullRobotModelFactory, SDFFullRobotModel estimatorModel, SDFFullRobotModel controllerModel,
            StateEstimationDataFromController stateEstimationDataFromControllerSink,
            StateEstimationDataFromController stateEstimationDataFromControllerSource)
      {
         this.fullRobotModelFactory = fullRobotModelFactory;
         this.estimatorModel = estimatorModel;
         this.controllerModel = controllerModel;
         this.stateEstimationDataFromControllerSink = stateEstimationDataFromControllerSink;
         this.stateEstimationDataFromControllerSource = stateEstimationDataFromControllerSource;
      }

      @Override
      public IntermediateControllerStateHolder newInstance()
      {
         return new IntermediateControllerStateHolder(fullRobotModelFactory, estimatorModel, controllerModel, stateEstimationDataFromControllerSink,
               stateEstimationDataFromControllerSource);
      }

   }
}
