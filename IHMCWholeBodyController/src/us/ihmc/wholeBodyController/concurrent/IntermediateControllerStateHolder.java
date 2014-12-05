package us.ihmc.wholeBodyController.concurrent;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.GenericCRC32;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointDesiredAccelerationChecksum;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointDesiredAccelerationCopier;
import us.ihmc.wholeBodyController.WholeBodyControlParameters;

public class IntermediateControllerStateHolder
{
   private final GenericCRC32 estimatorChecksumCalculator = new GenericCRC32();
   private final GenericCRC32 controllerChecksumCalculator = new GenericCRC32();
   private long checksum;
   
   private final InverseDynamicsJointDesiredAccelerationCopier controllerToIntermediateCopier;
   private final InverseDynamicsJointDesiredAccelerationCopier intermediateToEstimatorCopier;

   private final InverseDynamicsJointDesiredAccelerationChecksum controllerChecksum;
   private final InverseDynamicsJointDesiredAccelerationChecksum estimatorChecksum;
   
   public IntermediateControllerStateHolder(WholeBodyControlParameters wholeBodyControlParameters, SDFFullRobotModel estimatorModel,
         SDFFullRobotModel controllerModel)
   {
      SDFFullRobotModel intermediateModel = wholeBodyControlParameters.createFullRobotModel();

      controllerToIntermediateCopier = new InverseDynamicsJointDesiredAccelerationCopier(controllerModel.getElevator(), intermediateModel.getElevator());
      intermediateToEstimatorCopier = new InverseDynamicsJointDesiredAccelerationCopier(intermediateModel.getElevator(), estimatorModel.getElevator());
      
      controllerChecksum = new InverseDynamicsJointDesiredAccelerationChecksum(controllerModel.getElevator(), controllerChecksumCalculator);
      estimatorChecksum = new InverseDynamicsJointDesiredAccelerationChecksum(estimatorModel.getElevator(), estimatorChecksumCalculator);
   }

   public void setFromController()
   {
      checksum = calculateControllerChecksum();
      controllerToIntermediateCopier.copy();
   }

   public void getIntoEstimator()
   {
      intermediateToEstimatorCopier.copy();
   }
   
   private long calculateControllerChecksum()
   {
      controllerChecksumCalculator.reset();
      controllerChecksum.calculate();
      return controllerChecksumCalculator.getValue();
   }
   
   private long calculateEstimatorChecksum()
   {
      estimatorChecksumCalculator.reset();
      estimatorChecksum.calculate();
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

      private final SDFFullRobotModel estimatorModel;
      private final SDFFullRobotModel controllerModel;
      private final WholeBodyControlParameters wholeBodyControlParameters;

      public Builder(WholeBodyControlParameters wholeBodyControlParameters, SDFFullRobotModel estimatorModel, SDFFullRobotModel controllerModel)
      {
         this.wholeBodyControlParameters = wholeBodyControlParameters;
         this.estimatorModel = estimatorModel;
         this.controllerModel = controllerModel;
      }

      @Override
      public IntermediateControllerStateHolder newInstance()
      {
         return new IntermediateControllerStateHolder(wholeBodyControlParameters, estimatorModel, controllerModel);
      }

   }
}
