package us.ihmc.vicon;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;

public class ViconJavaInterface
{
   public static native boolean ViconConnect(String ip);

   public static native void ViconClose();

   public static native boolean ViconIsConnected();

   public static native String ViconGetLastError();

   public static native void ViconGetFrame();

   public static native int ViconGetNumMarkers();

   public static native String ViconGetMarkerName(int id);

   public static native Point3D ViconGetMarker(String markerName);

   public static native int ViconGetNumBodies();

   public static native String ViconGetBodyName(int id);

//   public static native Pose ViconGetBodyAngleAxis(String bodyName);
//
//   public static native Pose ViconGetBodyEulerAngles(String bodyName);

   public static native QuaternionPose ViconGetBodyQuaternion(String bodyName);

   public static native RotationMatrix ViconGetBodyRotationMatrix(String bodyName);

   public static native double ViconGetFrameTimeStamp();

   public static native int ViconGetNumLocalBodies();

//   public static native Pose ViconGetLocalBodyAngleAxis(String bodyName);
//
//   public static native Pose ViconGetLocalBodyEulerAngles(String bodyName);
//
//   public Pose getPose(String bodyName)
//   {
//      ViconGetFrame();
//
//      return ViconGetBodyAngleAxis(bodyName);
//   }

   public void test()
   {
      System.out.println("received test request");
   }


   static
   {
      try
      {
         System.loadLibrary("Vicon");
      }
      catch (UnsatisfiedLinkError e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      ViconConnect("10.4.1.100");

      boolean connected = ViconIsConnected();
      if (connected)
         System.out.println("successfully connected");
      else
         System.out.println("not connected");

      ViconGetFrame();

//    int numMarkers = ViconGetNumMarkers();
//    System.out.println("numMarkers=" + numMarkers);
//
//    int z = 0;
//    while(z == 0)
//    {
//        for (int i = 0; i < numMarkers; i++)
//        {
//            String name = ViconGetMarkerName(i);
//            Point3D position = ViconGetMarker(name);
//            System.out.println("marker " + i + " is " + name + ": " + position);
//        }
//
//        try
//        {
//            Thread.sleep(1000);
//        }
//        catch (InterruptedException ex)
//        {
//        }
//    }
//    int numBodies = ViconGetNumBodies();
//    System.out.println("numBodies=" + numBodies);
//
//    for(int i=0; i < numBodies; i++)
//    {
//        String name = ViconGetBodyName(i);
//        Pose pose = ViconGetBodyAngleAxis(name);
//        System.out.println("body " + i + " is " + name + ": " + pose);
//    }


      @SuppressWarnings("unused")
      long start = System.currentTimeMillis();
//      int count = 0;
      String name = ViconGetBodyName(0);
      while (true)
      {
         long startTime = System.currentTimeMillis();
         ViconGetFrame();
         QuaternionPose pose = ViconGetBodyQuaternion(name);
         long endTime = System.currentTimeMillis();
//         count++;
         System.out.println(name + ": " + pose + " in " + (endTime-startTime) + " ms");

//       if(count >= 20)
//       {
//           long now = System.currentTimeMillis();
//           System.out.println("read " + count + " in " + (now - start));
//           start = now;
//           count = 0;
//       }


         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException ex)
         {
         }
      }

//    Pose poseEuler = ViconGetBodyEulerAngles("one");
//    System.out.println("poseE = " + poseEuler);
//
//    Pose poseQuaternion = ViconGetBodyQuaternion("one");
//    System.out.println("poseQ = " + poseQuaternion);

//    ViconJavaInterface.ViconClose();
   }
}
