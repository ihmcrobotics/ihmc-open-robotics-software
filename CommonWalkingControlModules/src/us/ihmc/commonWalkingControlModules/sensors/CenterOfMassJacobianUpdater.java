package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import us.ihmc.simulationconstructionset.robotController.SensorProcessor;

public class CenterOfMassJacobianUpdater implements SensorProcessor
{
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
