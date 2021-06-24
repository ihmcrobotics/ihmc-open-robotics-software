package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.VRControllerState;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXTools;

import java.nio.IntBuffer;

/**
 * Represents a tracked VR device such as the head mounted
 * display, wands etc.
 */
public class GDXVRDevice
{
   private final GDXVRContext context;
   private final GDXVRDeviceType type;
   private final GDXVRControllerRole role;
   private long buttons = 0;
   private final VRControllerState state = VRControllerState.create();
   private final ModelInstance modelInstance;
   private final String name;
   private final IntBuffer tempIntBuffer = BufferUtils.newIntBuffer(1);
   private final Vector3 velocity = new Vector3();
   private final Vector3 angularVelocity = new Vector3();
   private final FramePose3D pose = new FramePose3D();
   private final RotationMatrix tempRotationMatrix = new RotationMatrix();
   private boolean isValid;
   private final int deviceIndex;

   public GDXVRDevice(GDXVRContext context, int deviceIndex, GDXVRDeviceType type, GDXVRControllerRole role)
   {
      this.context = context;
      this.deviceIndex = deviceIndex;
      this.type = type;
      this.role = role;
      Model model = context.loadRenderModel(getStringProperty(GDXVRDeviceProperty.RenderModelName_String));
      this.modelInstance = model != null ? new ModelInstance(model) : null;

      String roleName = role == GDXVRControllerRole.LeftHand ? role.name() : "";
      roleName += role == GDXVRControllerRole.RightHand ? role.name() : "";
      name = type.name() + roleName;
   }

   public void updatePoseInTrackerFrame(HmdMatrix34 openVRRigidBodyTransform)
   {
      pose.setReferenceFrame(context.getVRPlayAreaFrame());
      GDXTools.toEuclid(openVRRigidBodyTransform, pose);

      // update model instance
      if (modelInstance != null)
         getPose(ReferenceFrame.getWorldFrame(), modelInstance.transform);
   }

   public void getPose(ReferenceFrame referenceFrame, Matrix4 gdxRigidBodyTransform)
   {
      pose.changeFrame(referenceFrame);
      pose.getOrientation().get(tempRotationMatrix);
      GDXTools.toGDX(tempRotationMatrix, gdxRigidBodyTransform);
      GDXTools.toGDX(pose.getPosition(), gdxRigidBodyTransform);
   }

   public FramePose3D getPose()
   {
      return pose;
   }

   /**
    * @return the {@link GDXVRDeviceType}
    */
   public GDXVRDeviceType getType()
   {
      return type;
   }

   /**
    * The {@link GDXVRControllerRole}, indicating if the {@link GDXVRDevice} is assigned
    * to the left or right hand.
    *
    * <p>
    * <strong>Note</strong>: the role is not reliable! If one controller is connected on
    * startup, it will have a role of {@link GDXVRControllerRole#Unknown} and retain
    * that role even if a second controller is connected (which will also haven an
    * unknown role). The role is only reliable if two controllers are connected
    * already, and none of the controllers disconnects during the application
    * life-time.</br>
    * At least on the HTC Vive, the first connected controller is always the right hand
    * and the second connected controller is the left hand. The order stays the same
    * even if controllers disconnect/reconnect during the application life-time.
    * </p>
    */
   // FIXME role might change as per API, but never saw it
   public GDXVRControllerRole getControllerRole()
   {
      return role;
   }

   /**
    * @return whether the device is connected
    */
   public boolean isConnected()
   {
      return VRSystem.VRSystem_IsTrackedDeviceConnected(deviceIndex);
   }

   /**
    * @return whether the button from {@link GDXVRControllerButtons} is pressed
    */
   public boolean isButtonPressed(int button)
   {
      if (button < 0 || button >= 64)
         return false;
      return (buttons & (1L << button)) != 0;
   }

   void setButton(int button, boolean pressed)
   {
      if (pressed)
      {
         buttons |= (1L << button);
      }
      else
      {
         buttons ^= (1L << button);
      }
   }

   /**
    * @return the x-coordinate in the range [-1, 1] of the given axis from {@link GDXVRControllerAxes}
    */
   public float getAxisX(int axis)
   {
      if (axis < 0 || axis >= 5)
         return 0;
      VRSystem.VRSystem_GetControllerState(deviceIndex, state);
      return state.rAxis(axis).x();
   }

   /**
    * @return the y-coordinate in the range [-1, 1] of the given axis from {@link GDXVRControllerAxes}
    */
   public float getAxisY(int axis)
   {
      if (axis < 0 || axis >= 5)
         return 0;
      VRSystem.VRSystem_GetControllerState(deviceIndex, state);
      return state.rAxis(axis).y();
   }

   /**
    * Trigger a haptic pulse (vibrate) for the duration in microseconds. Subsequent calls
    * to this method within 5ms will be ignored.
    *
    * @param duration pulse duration in microseconds
    */
   public void triggerHapticPulse(short duration)
   {
      VRSystem.VRSystem_TriggerHapticPulse(deviceIndex, 0, duration);
   }

   /**
    * @return a boolean property or false if the query failed
    */
   public boolean getBooleanProperty(GDXVRDeviceProperty property)
   {
      tempIntBuffer.put(0, 0);
      boolean result = VRSystem.VRSystem_GetBoolTrackedDeviceProperty(deviceIndex, property.value, tempIntBuffer);
      if (tempIntBuffer.get(0) != 0)
         return false;
      else
         return result;
   }

   /**
    * @return a float property or 0 if the query failed
    */
   public float getFloatProperty(GDXVRDeviceProperty property)
   {
      tempIntBuffer.put(0, 0);
      float result = VRSystem.VRSystem_GetFloatTrackedDeviceProperty(deviceIndex, property.value, tempIntBuffer);
      if (tempIntBuffer.get(0) != 0)
         return 0;
      else
         return result;
   }

   /**
    * @return an int property or 0 if the query failed
    */
   public int getInt32Property(GDXVRDeviceProperty property)
   {
      tempIntBuffer.put(0, 0);
      int result = VRSystem.VRSystem_GetInt32TrackedDeviceProperty(deviceIndex, property.value, tempIntBuffer);
      if (tempIntBuffer.get(0) != 0)
         return 0;
      else
         return result;
   }

   /**
    * @return a long property or 0 if the query failed
    */
   public long getUInt64Property(GDXVRDeviceProperty property)
   {
      tempIntBuffer.put(0, 0);
      long result = VRSystem.VRSystem_GetUint64TrackedDeviceProperty(deviceIndex, property.value, tempIntBuffer);
      if (tempIntBuffer.get(0) != 0)
         return 0;
      else
         return result;
   }

   /**
    * @return a string property or null if the query failed
    */
   public String getStringProperty(GDXVRDeviceProperty property)
   {
      tempIntBuffer.put(0, 0);

      String result = VRSystem.VRSystem_GetStringTrackedDeviceProperty(deviceIndex, property.value, tempIntBuffer);
      if (tempIntBuffer.get(0) != 0)
         return null;
      return result;
   }

   /**
    * @return a {@link ModelInstance} with the transform updated to the latest tracked position and orientation in world space for rendering or null
    */
   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public int getDeviceIndex()
   {
      return deviceIndex;
   }

   /**
    * the velocity in m/s in tracker space space
    **/
   public Vector3 getVelocity()
   {
      return velocity;
   }

   /**
    * the angular velocity in radians/s in tracker space
    **/
   public Vector3 getAngularVelocity()
   {
      return angularVelocity;
   }

   /**
    * whether the pose is valid our invalid, e.g. outdated because of tracking failure
    **/
   public boolean isValid()
   {
      return isValid;
   }

   public void setValid(boolean valid)
   {
      isValid = valid;
   }

   @Override
   public String toString()
   {
      return "VRDevice[manufacturer=" + getStringProperty(GDXVRDeviceProperty.ManufacturerName_String) + ", renderModel=" + getStringProperty(
            GDXVRDeviceProperty.RenderModelName_String) + ", deviceIndex=" + deviceIndex + ", type=" + type + ", role=" + role + "]";
   }
}
