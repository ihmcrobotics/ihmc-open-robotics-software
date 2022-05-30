package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;
import org.bytedeco.spinnaker.global.Spinnaker_C;

import static us.ihmc.perception.spinnaker.SpinnakerTools.assertNoError;

public class SpinnakerHardwareManager
{
   private final spinSystem spinSystem = new spinSystem();
   private final spinCameraList spinCameraList = new spinCameraList();

   public SpinnakerHardwareManager()
   {
      assertNoError(Spinnaker_C.spinSystemGetInstance(spinSystem), "Unable to retrieve Spinnaker system instance!");
      assertNoError(Spinnaker_C.spinCameraListCreateEmpty(spinCameraList), "Unable to create camera list");
      assertNoError(Spinnaker_C.spinSystemGetCameras(spinSystem, spinCameraList), "Unable to retrieve camera list from Spinnaker system");
   }

   public SpinnakerBlackfly buildBlackfly(String serialNumber)
   {
      return buildBlackfly(serialNumber, "Continuous");
   }

   // acquisitionMode = Single/Multi(?)/Continuous. Should be continuous in almost all cases
   public SpinnakerBlackfly buildBlackfly(String serialNumber, String acquisitionMode)
   {
      spinCamera blackflyCamera = new spinCamera();
      assertNoError(Spinnaker_C.spinCameraListGetBySerial(spinCameraList, new BytePointer(serialNumber), blackflyCamera),
                    "Unable to create spinCamera from serial number!");
      // Note: the SpinnakerBlackfly class is responsible for releasing the new camera. This is done with the destroy() call
      return new SpinnakerBlackfly(blackflyCamera, acquisitionMode, serialNumber);
   }

   /**
    * The Spinnaker hardware manager should not be destroyed prior to the destruction of all cameras.
    */
   public void destroy()
   {
      Spinnaker_C.spinCameraListClear(spinCameraList);
      Spinnaker_C.spinCameraListDestroy(spinCameraList);
      Spinnaker_C.spinSystemReleaseInstance(spinSystem);
   }
}
