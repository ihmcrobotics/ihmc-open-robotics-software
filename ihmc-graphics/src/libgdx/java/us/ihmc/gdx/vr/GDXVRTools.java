package us.ihmc.gdx.vr;

import com.badlogic.gdx.math.Matrix4;
import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;
import org.lwjgl.openvr.VR;

import java.nio.FloatBuffer;

public class GDXVRTools
{
   /** device index of the head mounted display */
   public static final int HMD_DEVICE_INDEX = VR.k_unTrackedDeviceIndex_Hmd;
   /** maximum device index */
   public static final int MAX_DEVICE_INDEX = VR.k_unMaxTrackedDeviceCount - 1;

   public static void hmdMat4toMatrix4(HmdMatrix44 hdm, Matrix4 mat)
   {
      float[] val = mat.val;
      FloatBuffer m = hdm.m();

      val[0] = m.get(0);
      val[1] = m.get(4);
      val[2] = m.get(8);
      val[3] = m.get(12);

      val[4] = m.get(1);
      val[5] = m.get(5);
      val[6] = m.get(9);
      val[7] = m.get(13);

      val[8] = m.get(2);
      val[9] = m.get(6);
      val[10] = m.get(10);
      val[11] = m.get(14);

      val[12] = m.get(3);
      val[13] = m.get(7);
      val[14] = m.get(11);
      val[15] = m.get(15);
   }

   public static void hmdMat34ToMatrix4(HmdMatrix34 hmd, Matrix4 mat)
   {
      float[] val = mat.val;
      FloatBuffer m = hmd.m();

      val[0] = m.get(0);
      val[1] = m.get(4);
      val[2] = m.get(8);
      val[3] = 0;

      val[4] = m.get(1);
      val[5] = m.get(5);
      val[6] = m.get(9);
      val[7] = 0;

      val[8] = m.get(2);
      val[9] = m.get(6);
      val[10] = m.get(10);
      val[11] = 0;

      val[12] = m.get(3);
      val[13] = m.get(7);
      val[14] = m.get(11);
      val[15] = 1;
   }
}
