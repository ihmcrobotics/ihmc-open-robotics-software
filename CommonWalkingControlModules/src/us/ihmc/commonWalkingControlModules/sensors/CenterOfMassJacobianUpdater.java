package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.SensorProcessor;

public class CenterOfMassJacobianUpdater implements SensorProcessor
{
   private static final long serialVersionUID = -1337712480289940657L;
   private final String name = getClass().getSimpleName();
   private final CenterOfMassJacobian centerOfMassJacobian;

   public CenterOfMassJacobianUpdater(CenterOfMassJacobian centerOfMassJacobian)
   {
      this.centerOfMassJacobian = centerOfMassJacobian;
   }

   public void initialize()
   {
      update();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void update()
   {
      centerOfMassJacobian.compute();
   }
}
