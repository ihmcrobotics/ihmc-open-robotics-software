package us.ihmc.robotics.saveableModule;

import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;

public class SaveableModuleStateTools
{

   public static void registerYoTuple3DToSave(YoTuple3D framePoint3D, SaveableModuleState state)
   {
      state.registerDoubleToSave(framePoint3D.getYoX());
      state.registerDoubleToSave(framePoint3D.getYoY());
      state.registerDoubleToSave(framePoint3D.getYoZ());
   }

   public static void registerYoTuple2DToSave(YoTuple2D framePoint2D, SaveableModuleState state)
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
      registerYoTuple3DToSave(framePose3D.getPosition(), state);
      registerYoFrameQuaternionToSave(framePose3D.getOrientation(), state);
   }

}
