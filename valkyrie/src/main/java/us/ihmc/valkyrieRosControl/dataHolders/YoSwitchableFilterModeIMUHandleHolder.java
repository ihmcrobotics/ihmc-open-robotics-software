package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.rosControl.wholeRobot.IMUHandle;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainData;

/**
 * Created by dstephen on 12/14/15.
 */
public class YoSwitchableFilterModeIMUHandleHolder extends YoIMUHandleHolder
{
   private final YoEnum<MicroStrainData.MicrostrainFilterType> filterTypeToUse;

   public static YoSwitchableFilterModeIMUHandleHolder create(IMUHandle complimentaryFilterHandle, IMUHandle kalmanFilterHandle, IMUDefinition definition, YoVariableRegistry parentRegistry)
   {
      return new YoSwitchableFilterModeIMUHandleHolder(new SwitchableFilterModeIMUHandle(definition.getName(), complimentaryFilterHandle, kalmanFilterHandle), definition, parentRegistry);
   }

   private YoSwitchableFilterModeIMUHandleHolder(final SwitchableFilterModeIMUHandle handle, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      super(handle, imuDefinition, parentRegistry);

      filterTypeToUse = new YoEnum<>(handle.getName() + "_filterTypeToUse", parentRegistry, MicroStrainData.MicrostrainFilterType.class);

      filterTypeToUse.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            handle.setFilterTypeToUse(filterTypeToUse.getEnumValue());
         }
      });
      filterTypeToUse.set(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);
   }
}
