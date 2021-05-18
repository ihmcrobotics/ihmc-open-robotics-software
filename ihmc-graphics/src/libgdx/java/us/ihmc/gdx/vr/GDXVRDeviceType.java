package us.ihmc.gdx.vr;

/**
 * Type of a {@link GDXVRDevice}
 */
public enum GDXVRDeviceType
{
   /**
    * the head mounted display
    **/
   HeadMountedDisplay,
   /**
    * a controller like Oculus touch or HTC Vice controller
    **/
   Controller,
   /**
    * a camera/base station tracking the HMD and/or controllers
    **/
   BaseStation,
   /**
    * a generic VR tracking device
    **/
   Generic
}
