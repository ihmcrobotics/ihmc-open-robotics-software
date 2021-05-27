package us.ihmc.gdx.vr;

public enum GDXVRDeviceProperty
{
   Invalid(0),

   // general properties that apply to all device classes
   TrackingSystemName_String(1000),
   ModelNumber_String(1001),
   SerialNumber_String(1002),
   RenderModelName_String(1003),
   WillDriftInYaw_Bool(1004),
   ManufacturerName_String(1005),
   TrackingFirmwareVersion_String(1006),
   HardwareRevision_String(1007),
   AllWirelessDongleDescriptions_String(1008),
   ConnectedWirelessDongle_String(1009),
   DeviceIsWireless_Bool(1010),
   DeviceIsCharging_Bool(1011),
   DeviceBatteryPercentage_Float(1012),
   // 0 is empty), 1 is full
   // FIXME
   // StatusDisplayTransform_Matrix34(1013),
   Firmware_UpdateAvailable_Bool(1014),
   Firmware_ManualUpdate_Bool(1015),
   Firmware_ManualUpdateURL_String(1016),
   HardwareRevision_Uint64(1017),
   FirmwareVersion_Uint64(1018),
   FPGAVersion_Uint64(1019),
   VRCVersion_Uint64(1020),
   RadioVersion_Uint64(1021),
   DongleVersion_Uint64(1022),
   BlockServerShutdown_Bool(1023),
   CanUnifyCoordinateSystemWithHmd_Bool(1024),
   ContainsProximitySensor_Bool(1025),
   DeviceProvidesBatteryStatus_Bool(1026),
   DeviceCanPowerOff_Bool(1027),
   Firmware_ProgrammingTarget_String(1028),
   DeviceClass_Int32(1029),
   HasCamera_Bool(1030),
   DriverVersion_String(1031),
   Firmware_ForceUpdateRequired_Bool(1032),
   ViveSystemButtonFixRequired_Bool(1033),

   // Properties that are unique to TrackedDeviceClass_HMD
   ReportsTimeSinceVSync_Bool(2000),
   SecondsFromVsyncToPhotons_Float(2001),
   DisplayFrequency_Float(2002),
   UserIpdMeters_Float(2003),
   CurrentUniverseId_Uint64(2004),
   PreviousUniverseId_Uint64(2005),
   DisplayFirmwareVersion_Uint64(2006),
   IsOnDesktop_Bool(2007),
   DisplayMCType_Int32(2008),
   DisplayMCOffset_Float(2009),
   DisplayMCScale_Float(2010),
   EdidVendorID_Int32(2011),
   DisplayMCImageLeft_String(2012),
   DisplayMCImageRight_String(2013),
   DisplayGCBlackClamp_Float(2014),
   EdidProductID_Int32(2015),
   // FIXME
   // CameraToHeadTransform_Matrix34(2016),
   DisplayGCType_Int32(2017),
   DisplayGCOffset_Float(2018),
   DisplayGCScale_Float(2019),
   DisplayGCPrescale_Float(2020),
   DisplayGCImage_String(2021),
   LensCenterLeftU_Float(2022),
   LensCenterLeftV_Float(2023),
   LensCenterRightU_Float(2024),
   LensCenterRightV_Float(2025),
   UserHeadToEyeDepthMeters_Float(2026),
   CameraFirmwareVersion_Uint64(2027),
   CameraFirmwareDescription_String(2028),
   DisplayFPGAVersion_Uint64(2029),
   DisplayBootloaderVersion_Uint64(2030),
   DisplayHardwareVersion_Uint64(2031),
   AudioFirmwareVersion_Uint64(2032),
   CameraCompatibilityMode_Int32(2033),
   ScreenshotHorizontalFieldOfViewDegrees_Float(2034),
   ScreenshotVerticalFieldOfViewDegrees_Float(2035),
   DisplaySuppressed_Bool(2036),
   DisplayAllowNightMode_Bool(2037),

   // Properties that are unique to TrackedDeviceClass_Controller
   AttachedDeviceId_String(3000),
   SupportedButtons_Uint64(3001),
   Axis0Type_Int32(3002), // Return value is of type EVRControllerAxisType
   Axis1Type_Int32(3003), // Return value is of type EVRControllerAxisType
   Axis2Type_Int32(3004), // Return value is of type EVRControllerAxisType
   Axis3Type_Int32(3005), // Return value is of type EVRControllerAxisType
   Axis4Type_Int32(3006), // Return value is of type EVRControllerAxisType
   ControllerRoleHint_Int32(3007), // Return value is of type ETrackedControllerRole

   // Properties that are unique to TrackedDeviceClass_TrackingReference
   FieldOfViewLeftDegrees_Float(4000),
   FieldOfViewRightDegrees_Float(4001),
   FieldOfViewTopDegrees_Float(4002),
   FieldOfViewBottomDegrees_Float(4003),
   TrackingRangeMinimumMeters_Float(4004),
   TrackingRangeMaximumMeters_Float(4005),
   ModeLabel_String(4006),

   // Properties that are used for user interface like icons names
   IconPathName_String(5000), // usually a directory named "icons"
   NamedIconPathDeviceOff_String(5001), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceSearching_String(5002), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceSearchingAlert_String(5003), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceReady_String(5004), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceReadyAlert_String(5005), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceNotReady_String(5006), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceStandby_String(5007), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others
   NamedIconPathDeviceAlertLow_String(5008), // PNG for static icon), or GIF for animation), 50x32 for headsets and 32x32 for others

   // Vendors are free to expose private debug data in this reserved region
   VendorSpecific_Reserved_Start(10000),
   VendorSpecific_Reserved_End(10999);

   public final int value;

   GDXVRDeviceProperty(int value)
   {
      this.value = value;
   }
}
