package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;
import us.ihmc.log.LogTools;

import static org.bytedeco.spinnaker.global.Spinnaker_C.*;
import static us.ihmc.perception.spinnaker.SpinnakerBlackflyTools.printOnError;

public class SpinnakerBlackflyDebug
{
   private final static int MAX_BUFF_LEN = 256;

   public static void printAllConnectedDevicesInformation()
   {
      spinSystem system = new spinSystem();
      printOnError(spinSystemGetInstance(system), "Unable to retrieve Spinnaker system instance!");

      spinCameraList cameras = new spinCameraList();
      printOnError(spinCameraListCreateEmpty(cameras), "Unable to create camera list");
      printOnError(spinSystemGetCameras(system, cameras), "Unable to retrieve camera list from Spinnaker system");

      SizeTPointer numberOfCameras = new SizeTPointer(1);
      printOnError(spinCameraListGetSize(cameras, numberOfCameras), "");

      for (int i = 0; i < numberOfCameras.get(); i++)
      {
         spinCamera spinCamera = new spinCamera();

         printOnError(spinCameraListGet(cameras, i, spinCamera), "");

         spinNodeMapHandle spinNodeMapHandle = new spinNodeMapHandle();
         printOnError(spinCameraGetTLDeviceNodeMap(spinCamera, spinNodeMapHandle), "");

         spinNodeHandle deviceInformation = new spinNodeHandle();
         printOnError(spinNodeMapGetNode(spinNodeMapHandle, new BytePointer("DeviceInformation"), deviceInformation), "");

         // Retrieve number of nodes within device information node
         SizeTPointer numFeatures = new SizeTPointer(1);
         if (isAvailableAndReadable(deviceInformation, "DeviceInformation"))
         {
            printOnError(spinCategoryGetNumFeatures(deviceInformation, numFeatures), "Unable to retrieve number of nodes.");
         }

         // Iterate through nodes and print information
         for (int j = 0; j < numFeatures.get(); j++)
         {
            spinNodeHandle spinNodeFeatureNode = new spinNodeHandle();
            printOnError(spinCategoryGetFeatureByIndex(deviceInformation, j, spinNodeFeatureNode), "Unable to retrieve node.");

            // get feature node name
            BytePointer featureName = new BytePointer(MAX_BUFF_LEN);
            SizeTPointer lenFeatureName = new SizeTPointer(1);
            lenFeatureName.put(MAX_BUFF_LEN);
            printOnError(spinNodeGetName(spinNodeFeatureNode, featureName, lenFeatureName), "Error retrieving node name.");

            int[] featureType = {spinNodeType.UnknownNode.value};
            if (isAvailableAndReadable(spinNodeFeatureNode, featureName.getString()))
            {
               printOnError(spinNodeGetType(spinNodeFeatureNode, featureType), "Unable to retrieve node type.");
            }
            else
            {
               System.out.println(featureName + ": Node not readable");
               continue;
            }
            BytePointer featureValue = new BytePointer(MAX_BUFF_LEN);
            SizeTPointer lenFeatureValue = new SizeTPointer(1);
            lenFeatureValue.put(MAX_BUFF_LEN);
            printOnError(spinNodeToString(spinNodeFeatureNode, featureValue, lenFeatureValue), "spinNodeToString");
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
      printOnError(spinNodeIsAvailable(spinNodeHandle, booleanAvailable), "Retrieving " + nodeName + " node availability");

      BytePointer booleanReadable = new BytePointer(1);
      printOnError(spinNodeIsReadable(spinNodeHandle, booleanReadable), "Retrieving " + nodeName + " node readability");
      return booleanReadable.getBool() && booleanAvailable.getBool();
   }

   public static void main(String[] args)
   {
      printAllConnectedDevicesInformation();
   }
}
