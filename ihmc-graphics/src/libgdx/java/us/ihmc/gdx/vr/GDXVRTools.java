package us.ihmc.gdx.vr;

import org.lwjgl.openvr.VR;

import java.nio.FloatBuffer;

public class GDXVRTools
{
   /** device index of the head mounted display */
   public static final int HMD_DEVICE_INDEX = VR.k_unTrackedDeviceIndex_Hmd;
   /** maximum device index */
   public static final int MAX_DEVICE_INDEX = VR.k_unMaxTrackedDeviceCount - 1;
}
