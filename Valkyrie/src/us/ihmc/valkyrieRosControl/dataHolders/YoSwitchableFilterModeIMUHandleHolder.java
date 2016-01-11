package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.rosControl.valkyrie.IMUHandle;
import us.ihmc.valkyrie.imu.MicroStrainData;

/**
 * Created by dstephen on 12/14/15.
 */
public class YoSwitchableFilterModeIMUHandleHolder extends YoIMUHandleHolder
{
   private final SwitchableFilterModeIMUHandle handle;
   private final EnumYoVariable<MicroStrainData.MicrostrainFilterType> filterTypeToUse;

   public static YoSwitchableFilterModeIMUHandleHolder create(IMUHandle complimentaryFilterHandle, IMUHandle kalmanFilterHandle, IMUDefinition definition, YoVariableRegistry parentRegistry)
   {
      return new YoSwitchableFilterModeIMUHandleHolder(new SwitchableFilterModeIMUHandle(definition.getName(), complimentaryFilterHandle, kalmanFilterHandle), definition, parentRegistry);
   }

   private YoSwitchableFilterModeIMUHandleHolder(final SwitchableFilterModeIMUHandle handle, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      super(handle, imuDefinition, parentRegistry);

      this.handle = handle;

      filterTypeToUse = new EnumYoVariable<>(handle.getName() + "_filterTypeToUse", parentRegistry, MicroStrainData.MicrostrainFilterType.class);

      filterTypeToUse.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            handle.setFilterTypeToUse(filterTypeToUse.getEnumValue());
         }
      });
      filterTypeToUse.set(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);
   }
}
