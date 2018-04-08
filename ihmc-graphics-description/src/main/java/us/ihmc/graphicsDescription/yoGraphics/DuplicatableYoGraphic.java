package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This interface allows YoGraphics to be duplicated onto another registry.
 * 
 * This allows implementing safer threading by creating a YoVariableRegistry for the GUI thread that is mirrored from the controller.
 * 
 * @author Jesper Smith
 *
 */
public interface DuplicatableYoGraphic
{
   @SuppressWarnings("unchecked")
   default <T extends YoVariable<?>> T getVariableInTargetRegistry(T original, YoVariableRegistry targetRegistry)
   {
      YoVariable<?> targetVariable = targetRegistry.getVariable(original.getFullNameWithNameSpace());
      if (targetVariable == null)
      {
         throw new RuntimeException("Cannot find variable with name " + original.getFullNameWithNameSpace() + " in target variable registry");
      }
      return (T) targetVariable;
   }

   default YoFramePoint3D createYoFramePointInTargetRegistry(YoFramePoint3D original, YoVariableRegistry targetRegistry)
   {
      YoDouble x = getVariableInTargetRegistry(original.getYoX(), targetRegistry);
      YoDouble y = getVariableInTargetRegistry(original.getYoY(), targetRegistry);
      YoDouble z = getVariableInTargetRegistry(original.getYoZ(), targetRegistry);
      return new YoFramePoint3D(x, y, z, original.getReferenceFrame());
   }

   default YoFrameYawPitchRoll createYoFrameOrientationInTargetRegistry(YoFrameYawPitchRoll original, YoVariableRegistry targetRegistry)
   {
      YoDouble yaw = getVariableInTargetRegistry(original.getYaw(), targetRegistry);
      YoDouble pitch = getVariableInTargetRegistry(original.getPitch(), targetRegistry);
      YoDouble roll = getVariableInTargetRegistry(original.getRoll(), targetRegistry);
      return new YoFrameYawPitchRoll(yaw, pitch, roll, original.getReferenceFrame());
   }

   YoGraphic duplicateOntoRegistry(YoVariableRegistry targetRegistry);
}
