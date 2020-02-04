package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.simulationconstructionset.util.RobotController;

import java.util.List;

public interface SphereControllerInterface extends RobotController
{
   void solveForTrajectory(List<? extends ContactStateProvider> stateProviders);
}
