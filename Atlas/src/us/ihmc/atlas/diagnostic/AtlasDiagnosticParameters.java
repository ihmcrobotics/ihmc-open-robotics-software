package us.ihmc.atlas.diagnostic;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;

public class AtlasDiagnosticParameters extends DiagnosticParameters
{
   private final DRCRobotModel robotModel;

   public AtlasDiagnosticParameters(DiagnosticEnvironment diagnosticEnvironment, DRCRobotModel robotModel)
   {
      super(diagnosticEnvironment);

      this.robotModel = robotModel;
   }

   @Override
   public String getPelvisIMUName()
   {
      return robotModel.getSensorInformation().getPrimaryBodyImu();
   }
}
