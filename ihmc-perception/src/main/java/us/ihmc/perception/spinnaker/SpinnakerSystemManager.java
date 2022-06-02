package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

import static us.ihmc.perception.spinnaker.SpinnakerTools.assertNoError;

public class SpinnakerSystemManager
{
   private final spinSystem spinSystem = new spinSystem();
   private final spinCameraList spinCameraList = new spinCameraList();
   private final ArrayList<Runnable> stuffToDestroy = new ArrayList<>();

   public SpinnakerSystemManager()
   {
      assertNoError(Spinnaker_C.spinSystemGetInstance(spinSystem), "Unable to retrieve Spinnaker system instance!");
      assertNoError(Spinnaker_C.spinCameraListCreateEmpty(spinCameraList), "Unable to create camera list");
      assertNoError(Spinnaker_C.spinSystemGetCameras(spinSystem, spinCameraList), "Unable to retrieve camera list from Spinnaker system");
   }

   public SpinnakerBlackfly createBlackfly(String serialNumber)
   {
      LogTools.info("Creating Blackfly with serial number {}", serialNumber);
      spinCamera blackflyCamera = new spinCamera();
      assertNoError(Spinnaker_C.spinCameraListGetBySerial(spinCameraList, new BytePointer(serialNumber), blackflyCamera),
                    "Unable to create spinCamera from serial number!");

      SpinnakerBlackfly spinnakerBlackfly = new SpinnakerBlackfly(blackflyCamera, serialNumber);
      stuffToDestroy.add(spinnakerBlackfly::destroy);
      return spinnakerBlackfly;
   }

   public void destroy()
   {
      for (Runnable destroy : stuffToDestroy)
      {
         destroy.run();
      }

      Spinnaker_C.spinCameraListClear(spinCameraList);
      Spinnaker_C.spinCameraListDestroy(spinCameraList);
      Spinnaker_C.spinSystemReleaseInstance(spinSystem);
   }
}
