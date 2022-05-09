package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;

import static org.bytedeco.spinnaker.global.Spinnaker_C.*;
import static us.ihmc.perception.spinnaker.SpinnakerTools.assertNoError;

public class SpinnakerHardwareManager
{
   private final spinSystem system;
   private final spinCameraList cameras;

   public SpinnakerHardwareManager()
   {
      system = new spinSystem();
      assertNoError(spinSystemGetInstance(system), "Unable to retrieve Spinnaker system instance!");

      cameras = new spinCameraList();
      assertNoError(spinCameraListCreateEmpty(cameras), "Unable to create camera list");
      assertNoError(spinSystemGetCameras(system, cameras), "Unable to retrieve camera list from Spinnaker system");
   }

   public BytedecoBlackfly buildBlackfly(String serial)
   {
      return buildBlackfly(serial, "Continuous");
   }

   public BytedecoBlackfly buildBlackfly(String serial, String acqMode) // acqMode = Single/Multi(?)/Continuous. Should be continuous in almost all cases
   {
      spinCamera blackflyCamera = new spinCamera();
      assertNoError(spinCameraListGetBySerial(cameras, new BytePointer(serial), blackflyCamera), "Unable to create spinCamera from serial number!");
      // Note: the BytedecoBlackfly class is responsible for releasing the new camera. This is done with the destroy() call
      return new BytedecoBlackfly(blackflyCamera, acqMode, serial);
   }

   /**
    * The Spinnaker hardware manager should not be destroyed prior to the destruction of all cameras.
    */
   public void destroy()
   {
      spinCameraListClear(cameras);
      spinCameraListDestroy(cameras);
      spinSystemReleaseInstance(system);
   }
}
