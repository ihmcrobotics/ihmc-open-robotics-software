package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinCamera;
import org.bytedeco.spinnaker.Spinnaker_C.spinCameraList;
import org.bytedeco.spinnaker.Spinnaker_C.spinSystem;
import org.bytedeco.spinnaker.global.Spinnaker_C;

import java.util.HashMap;
import java.util.Map;

import static org.bytedeco.spinnaker.global.Spinnaker_C.*;
import static us.ihmc.perception.spinnaker.SpinnakerBlackflyTools.assertNoError;

public class SpinnakerBlackflyManager
{
   private final spinSystem spinSystem = new spinSystem();
   private final spinCameraList spinCameraList = new spinCameraList();
   private final Map<String, SpinnakerBlackfly> serialNumberToBlackflyMap = new HashMap<>();

   public SpinnakerBlackflyManager()
   {
      assertNoError(Spinnaker_C.spinSystemGetInstance(spinSystem), "Unable to retrieve Spinnaker system instance!");
      assertNoError(Spinnaker_C.spinCameraListCreateEmpty(spinCameraList), "Unable to create camera list");
      assertNoError(Spinnaker_C.spinSystemGetCameras(spinSystem, spinCameraList), "Unable to retrieve camera list from Spinnaker system");
   }

   public SpinnakerBlackfly createSpinnakerBlackfly(String serialNumber)
   {
      spinCamera spinCamera = new spinCamera();
      assertNoError(spinCameraListGetBySerial(spinCameraList, new BytePointer(serialNumber), spinCamera), "Unable to create spinCamera from serial number!");
      SpinnakerBlackfly spinnakerBlackfly = new SpinnakerBlackfly(spinCamera, serialNumber);
      spinnakerBlackfly.setAcquisitionMode(Spinnaker_C.spinAcquisitionModeEnums.AcquisitionMode_Continuous);
      spinnakerBlackfly.setPixelFormat(Spinnaker_C.spinPixelFormatEnums.PixelFormat_BGR8);
      // We only want the newest image for the lowest latency possible
      spinnakerBlackfly.setBufferHandlingMode(spinTLStreamBufferHandlingModeEnums.StreamBufferHandlingMode_NewestOnly);
      spinnakerBlackfly.startAcquiringImages();
      serialNumberToBlackflyMap.put(serialNumber, spinnakerBlackfly);
      return spinnakerBlackfly;
   }

   public void destroy()
   {
      for (Map.Entry<String, SpinnakerBlackfly> entry : serialNumberToBlackflyMap.entrySet())
         spinCameraRelease(entry.getValue().getSpinCamera());
      spinCameraListClear(spinCameraList);
      spinCameraListDestroy(spinCameraList);
      spinSystemReleaseInstance(spinSystem);
   }
}
