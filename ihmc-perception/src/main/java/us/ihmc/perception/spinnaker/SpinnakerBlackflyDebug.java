package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;
import us.ihmc.log.LogTools;

import static org.bytedeco.spinnaker.global.Spinnaker_C.*;
import static us.ihmc.perception.spinnaker.SpinnakerBlackflyTools.assertNoError;

public class SpinnakerBlackflyDebug
{
   private final static int MAX_BUFF_LEN = 256;

   public static void printAllConnectedDevicesInformation()
   {
      spinSystem system = new spinSystem();
      assertNoError(spinSystemGetInstance(system), "Unable to retrieve Spinnaker system instance!");

      spinCameraList cameras = new spinCameraList();
      assertNoError(spinCameraListCreateEmpty(cameras), "Unable to create camera list");
      assertNoError(spinSystemGetCameras(system, cameras), "Unable to retrieve camera list from Spinnaker system");

      SizeTPointer numberOfCameras = new SizeTPointer(1);
      assertNoError(spinCameraListGetSize(cameras, numberOfCameras), "");

      for (int i = 0; i < numberOfCameras.get(); i++)
      {
         spinCamera spinCamera = new spinCamera();

         assertNoError(spinCameraListGet(cameras, i, spinCamera), "");

         spinNodeMapHandle spinNodeMapHandle = new spinNodeMapHandle();
         assertNoError(spinCameraGetTLDeviceNodeMap(spinCamera, spinNodeMapHandle), "");

         spinNodeHandle deviceInformation = new spinNodeHandle();
         assertNoError(spinNodeMapGetNode(spinNodeMapHandle, new BytePointer("DeviceInformation"), deviceInformation), "");

         // Retrieve number of nodes within device information node
         SizeTPointer numFeatures = new SizeTPointer(1);
         if (isAvailableAndReadable(deviceInformation, "DeviceInformation"))
         {
            assertNoError(spinCategoryGetNumFeatures(deviceInformation, numFeatures), "Unable to retrieve number of nodes.");
         }

         // Iterate through nodes and print information
         for (int j = 0; j < numFeatures.get(); j++)
         {
            spinNodeHandle spinNodeFeatureNode = new spinNodeHandle();
            assertNoError(spinCategoryGetFeatureByIndex(deviceInformation, j, spinNodeFeatureNode), "Unable to retrieve node.");

            // get feature node name
            BytePointer featureName = new BytePointer(MAX_BUFF_LEN);
            SizeTPointer lenFeatureName = new SizeTPointer(1);
            lenFeatureName.put(MAX_BUFF_LEN);
            assertNoError(spinNodeGetName(spinNodeFeatureNode, featureName, lenFeatureName), "Error retrieving node name.");

            int[] featureType = {spinNodeType.UnknownNode.value};
            if (isAvailableAndReadable(spinNodeFeatureNode, featureName.getString()))
            {
               assertNoError(spinNodeGetType(spinNodeFeatureNode, featureType), "Unable to retrieve node type.");
            }
            else
            {
               System.out.println(featureName + ": Node not readable");
               continue;
            }
            BytePointer featureValue = new BytePointer(MAX_BUFF_LEN);
            SizeTPointer lenFeatureValue = new SizeTPointer(1);
            lenFeatureValue.put(MAX_BUFF_LEN);
            assertNoError(spinNodeToString(spinNodeFeatureNode, featureValue, lenFeatureValue), "spinNodeToString");
            LogTools.info(featureName.getString().trim() + ": " + featureValue.getString().trim() + ".");
         }

         spinCameraRelease(spinCamera);
      }

      spinCameraListClear(cameras);
      spinCameraListDestroy(cameras);
      spinSystemReleaseInstance(system);
   }

   private static boolean isAvailableAndReadable(spinNodeHandle spinNodeHandle, String nodeName)
   {
      BytePointer booleanAvailable = new BytePointer(1);
      assertNoError(spinNodeIsAvailable(spinNodeHandle, booleanAvailable), "Retrieving " + nodeName + " node availability");

      BytePointer booleanReadable = new BytePointer(1);
      assertNoError(spinNodeIsReadable(spinNodeHandle, booleanReadable), "Retrieving " + nodeName + " node readability");
      return booleanReadable.getBool() && booleanAvailable.getBool();
   }

   public static void main(String[] args)
   {
      printAllConnectedDevicesInformation();
   }
}
