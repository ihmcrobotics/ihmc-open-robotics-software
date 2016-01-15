package us.ihmc.valkyrieRosControl.automatedDiagnosticControl;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.darpaRoboticsChallenge.logProcessor.DRCLogProcessor;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorHelper;
import us.ihmc.darpaRoboticsChallenge.logProcessor.diagnostic.DiagnosticAnalysisProcessor;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieLogDiagnostic extends DRCLogProcessor
{

   public ValkyrieLogDiagnostic() throws IOException
   {
      super();

      LogDataProcessorHelper logDataProcessorHelper = createLogDataProcessorHelper();
      DiagnosticAnalysisProcessor diagnostic = new DiagnosticAnalysisProcessor(logDataProcessorHelper, drcRobotModel);
      diagnostic.addJointFourierAnalyses("raw_qd_", "", "raw_tau_", "", "", "TauDesired");
      diagnostic.addForceTrackingDelayEstimators("raw_tau_", "", "", "TauDesired");
      diagnostic.addJointConsistencyCheckers("q_", "", "qd_", "");
      diagnostic.addIMUConsistencyCheckers("raw_q_", "", "filt_qd_w", "_sp0", true);
      diagnostic.addCenterOfMassConsistencyChecker();
      setLogDataProcessor(diagnostic);

      startLogger();
   }

   public static void main(String[] args) throws IOException
   {
      new ValkyrieLogDiagnostic();
   }

   @Override
   public DRCRobotModel createDRCRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }
}
