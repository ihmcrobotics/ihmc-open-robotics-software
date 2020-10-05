package us.ihmc.simpleWholeBodyWalking.simpleSphere;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.simulationconstructionset.util.RobotController;

import java.util.List;

public interface SimpleSphereControllerInterface extends RobotController
{
   void setFootstepPlan(List<Footstep> footstepList, List<FootstepTiming> footstepTimingList);
}
