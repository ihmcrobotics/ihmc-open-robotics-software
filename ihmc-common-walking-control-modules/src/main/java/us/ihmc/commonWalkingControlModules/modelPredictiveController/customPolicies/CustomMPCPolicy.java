package us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;

import java.util.List;

public interface CustomMPCPolicy
{
   MPCCommand<?> computeMPCCommand(MPCContactHandler contactHandler, List<PreviewWindowSegment> contactStateProviders, double omega);

   MPCCommand<?> getMPCCommand();
}
