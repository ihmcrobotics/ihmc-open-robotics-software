package us.ihmc.robotics.saveableModule;

import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;

public class SaveableModuleStateTools
{

   public static void registerYoFramePoint3DToSave(YoFramePoint3D framePoint3D, SaveableModuleState state)
   {
      state.registerDoubleToSave(framePoint3D.getYoX());
      state.registerDoubleToSave(framePoint3D.getYoY());
      state.registerDoubleToSave(framePoint3D.getYoZ());
   }

   public static void registerYoFramePoint2DToSave(YoFramePoint2D framePoint2D, SaveableModuleState state)
   {
      state.registerDoubleToSave(framePoint2D.getYoX());
      state.registerDoubleToSave(framePoint2D.getYoY());
   }

   public static void registerYoFrameQuaternionToSave(YoFrameQuaternion frameQuaternion, SaveableModuleState state)
   {
      state.registerDoubleToSave(frameQuaternion.getYoQs());
      state.registerDoubleToSave(frameQuaternion.getYoQx());
      state.registerDoubleToSave(frameQuaternion.getYoQy());
      state.registerDoubleToSave(frameQuaternion.getYoQz());
   }

   public static void registerYoFramePose3DToSave(YoFramePose3D framePose3D, SaveableModuleState state)
   {
      registerYoFramePoint3DToSave(framePose3D.getPosition(), state);
      registerYoFrameQuaternionToSave(framePose3D.getOrientation(), state);
   }

}
