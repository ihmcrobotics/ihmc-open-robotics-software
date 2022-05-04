package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;
import us.ihmc.log.LogTools;

import static org.bytedeco.spinnaker.global.Spinnaker_C.*;

public class SpinnakerHardwareManager
{
   private static SpinnakerHardwareManager instance = null;

   public static SpinnakerHardwareManager getInstance() {
      if (instance == null)
         instance = new SpinnakerHardwareManager();

      return instance;
   }

   private spinSystem system;
   private spinCameraList cameras;

   private SpinnakerHardwareManager() {
      system = new spinSystem();
      assertNoError(spinSystemGetInstance(system), "Unable to retrieve Spinnaker system instance!");

      cameras = new spinCameraList();
      assertNoError(spinCameraListCreateEmpty(cameras), "Unable to create camera list");
      assertNoError(spinSystemGetCameras(system, cameras), "Unable to retrieve camera list from Spinnaker system");
   }

   public BytedecoBlackfly buildBlackfly(String serial) {
      return buildBlackfly(serial, "Continuous");
   }

   public BytedecoBlackfly buildBlackfly(String serial, String acqMode) //acqMode = Single/Multi(?)/Continuous. Should be continuous in almost all cases
   {
      spinCamera blackflyCamera = new spinCamera();
      assertNoError(spinCameraListGetBySerial(cameras, new BytePointer(serial), blackflyCamera), "Unable to create spinCamera from serial number!");
      return new BytedecoBlackfly(blackflyCamera, acqMode, serial); //Note: the BytedecoBlackfly class is responsible for releasing the new camera. This is done with the destroy() call
   }

   private static void assertNoError(spinError error, String errorMessage) {
      if (error.value != spinError.SPINNAKER_ERR_SUCCESS.value) {
         LogTools.fatal(errorMessage);
         throw new RuntimeException(String.valueOf(error.value));
      }
   }

   public void destroy() {
      spinCameraListClear(cameras);
      spinCameraListDestroy(cameras);
      spinSystemReleaseInstance(system);
   }
}
