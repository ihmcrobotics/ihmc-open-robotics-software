package us.ihmc.atlas.diagnostic;

import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.parameters.AtlasStateEstimatorParameters;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticSensorProcessingConfiguration;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasDiagnosticStateEstimatorParameters extends AtlasStateEstimatorParameters
{
   private DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration;

   public AtlasDiagnosticStateEstimatorParameters(DRCRobotJointMap jointMap, AtlasSensorInformation sensorInformation,
         DiagnosticParameters diagnosticParameters, boolean runningOnRealRobot, double estimatorDT)
   {
      super(jointMap, sensorInformation, runningOnRealRobot, estimatorDT);
      diagnosticSensorProcessingConfiguration = new DiagnosticSensorProcessingConfiguration(diagnosticParameters, estimatorDT);
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      super.configureSensorProcessing(sensorProcessing);
      diagnosticSensorProcessingConfiguration.configureSensorProcessing(sensorProcessing);
   }

   public DiagnosticSensorProcessingConfiguration getDiagnosticSensorProcessingConfiguration()
   {
      return diagnosticSensorProcessingConfiguration;
   }
}
