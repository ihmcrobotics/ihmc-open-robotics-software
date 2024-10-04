/*
 * Copyright LWJGL. All rights reserved.
 * License terms: https://www.lwjgl.org/license
 */
package us.ihmc.rdx;

import org.lwjgl.openvr.*;
import org.lwjgl.system.*;
import us.ihmc.log.LogTools;

import java.nio.*;

import static org.lwjgl.openvr.VR.*;
import static org.lwjgl.openvr.VRSystem.*;
import static org.lwjgl.system.MemoryStack.*;

public class HelloOpenVR
{

   private HelloOpenVR()
   {
   }

   public static void main(String[] args)
   {
      LogTools.error("VR_IsRuntimeInstalled() = {}", VR_IsRuntimeInstalled());
      LogTools.error("VR_RuntimePath() = {}", VR_RuntimePath());
      LogTools.error("VR_IsHmdPresent() = {}", VR_IsHmdPresent());

      try (MemoryStack stack = stackPush())
      {
         IntBuffer peError = stack.mallocInt(1);

         int token = VR_InitInternal(peError, 0);
         if (peError.get(0) == 0)
         {
            try
            {
               OpenVR.create(token);

               LogTools.error("Model Number : {}",
                              VRSystem_GetStringTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, ETrackedDeviceProperty_Prop_ModelNumber_String, peError));
               LogTools.error("Serial Number: {}",
                              VRSystem_GetStringTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, ETrackedDeviceProperty_Prop_SerialNumber_String, peError));

               IntBuffer w = stack.mallocInt(1);
               IntBuffer h = stack.mallocInt(1);
               VRSystem_GetRecommendedRenderTargetSize(w, h);
               LogTools.error("Recommended width : {}", w.get(0));
               LogTools.error("Recommended height: {}", h.get(0));
            }
            finally
            {
               VR_ShutdownInternal();
            }
         }
         else
         {
            LogTools.info("INIT ERROR SYMBOL: {}", VR_GetVRInitErrorAsSymbol(peError.get(0)));
            LogTools.info("INIT ERROR  DESCR: {}", VR_GetVRInitErrorAsEnglishDescription(peError.get(0)));
         }
      }
   }
}