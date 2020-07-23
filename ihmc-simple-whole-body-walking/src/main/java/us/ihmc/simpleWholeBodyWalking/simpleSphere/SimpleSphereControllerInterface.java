package us.ihmc.simpleWholeBodyWalking.simpleSphere;

import us.ihmc.simulationconstructionset.util.RobotController;

import java.util.List;

public interface SimpleSphereControllerInterface extends RobotController
{
   void solveForTrajectory();
}
