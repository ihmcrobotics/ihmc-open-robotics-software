package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRControllerState;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXTools;

import java.nio.IntBuffer;
import java.util.TreeSet;
import java.util.function.Function;

/**
 * Represents a tracked VR device such as the head mounted
 * display, wands etc.
 */
public class GDXVRDevice
{
   private final int deviceClass;
   private final GDXVRControllerRole role;
   private long buttons = 0;
   private final VRControllerState controllerState = VRControllerState.create();
   private final ModelInstance modelInstance;
   private final IntBuffer errorCode = BufferUtils.newIntBuffer(1);
   private final Vector3 velocity = new Vector3();
   private final Vector3 angularVelocity = new Vector3();
   private final FramePose3D pose = new FramePose3D();
   private final RotationMatrix tempRotationMatrix = new RotationMatrix();
   private final int deviceIndex;
   private final TreeSet<Integer> buttonsPressedThisFrame = new TreeSet<>();
   private final TreeSet<Integer> buttonsReleasedThisFrame = new TreeSet<>();

   public GDXVRDevice(int deviceIndex, int deviceClass, GDXVRControllerRole role, Function<String, Model> modelLoader)
   {
      this.deviceIndex = deviceIndex;
      this.deviceClass = deviceClass;
      this.role = role;

      String renderModelName = VRSystem.VRSystem_GetStringTrackedDeviceProperty(deviceIndex, VR.ETrackedDeviceProperty_Prop_RenderModelName_String, errorCode);
      Model model = modelLoader.apply(renderModelName);
      modelInstance = model != null ? new ModelInstance(model) : null;
   }

   public void resetBeforeUpdate()
   {
      buttonsPressedThisFrame.clear();
      buttonsReleasedThisFrame.clear();
   }

   public void updateControllerState()
   {
      VRSystem.VRSystem_GetControllerState(deviceIndex, controllerState);
   }

   public VRControllerState getControllerState()
   {
      return controllerState;
   }

   public void updatePoseInTrackerFrame(HmdMatrix34 openVRRigidBodyTransform, ReferenceFrame vrPlayAreaFrame)
   {
      pose.setReferenceFrame(vrPlayAreaFrame);
      GDXTools.toEuclid(openVRRigidBodyTransform, pose);
//      GDXVRContext.openVRYUpToIHMCZUpSpace.getRotation().transform(pose.getOrientation());
//      pose.applyInverseTransform(GDXVRContext.openVRYUpToIHMCZUpSpace);
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      //      pose.appendTransform(GDXVRContext.openVRYUpToIHMCZUpSpace);

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

   public int getDeviceClass()
   {
      return deviceClass;
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

   /** package-private */ void setButton(int button, boolean pressed)
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

   public void setButtonPressed(int button)
   {
      buttonsPressedThisFrame.add(button);
   }

   public void setButtonReleased(int button)
   {
      buttonsReleasedThisFrame.add(button);
   }

   public boolean isButtonNewlyPressed(int button)
   {
      return buttonsPressedThisFrame.contains(button);
   }

   public boolean isButtonNewlyReleased(int button)
   {
      return buttonsReleasedThisFrame.contains(button);
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

   @Override
   public String toString()
   {
      String manufacturerName = VRSystem.VRSystem_GetStringTrackedDeviceProperty(deviceIndex,
                                                                                 VR.ETrackedDeviceProperty_Prop_ManufacturerName_String,
                                                                                 errorCode);
      String renderModelName = VRSystem.VRSystem_GetStringTrackedDeviceProperty(deviceIndex,
                                                                                VR.ETrackedDeviceProperty_Prop_RenderModelName_String,
                                                                                errorCode);
      return "VRDevice[manufacturer=" + manufacturerName
             + ", renderModel=" + renderModelName
             + ", deviceIndex=" + deviceIndex
             + ", type=" + deviceClass
             + ", role=" + role
             + "]";
   }
}
