package optiTrack;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class MocapFrameDataPacket
{
   public short messageID;
   public short payloadSize;
   public int frameNumber;
   public int numberOfDataSets;
   public int numberOfRigidBodies;
   public int rigidBodyId;
   public float posX;
   public float posY;
   public float posZ;
   public float rotX;
   public float rotY;
   public float rotZ;
   public float rotW;
   public int nRigidMarkers;

   public int numberOfUnidentifiedMarkers;
   public float uMarkerX;
   public float uMarkerY;
   public float uMarkerZ;

   public Vector3D[] markerPosition;
   public int[] markerIds;
   public float[] markerSizes;

   public int numberOfSkeletons;
   public float latency;
   public int numberOfLabeledMarkers;

   public float meanMarkerError;

   public boolean isTracked = true;

   public int type = 0;

   public int numberOfMarkers;

   private static MocapRigidBody rigidBody;

   private static boolean DEBUG = false;

   private static ArrayList<MocapRigidBody> listfOfRigidbodies;

   public static ArrayList<MocapRigidBody> createFromBytes(byte[] bytes) throws IOException
   {
      listfOfRigidbodies = new ArrayList<>();

      final MocapFrameDataPacket data = new MocapFrameDataPacket();
      final ByteBuffer buf = ByteBuffer.wrap(bytes);

      buf.order(ByteOrder.LITTLE_ENDIAN);

      data.messageID = buf.getShort();
      data.payloadSize = buf.getShort();

      if (DEBUG)
      {
         System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>START MESSAGE");
         System.out.println("MOCAP Info:");
         System.out.println("Mssg ID: " + data.messageID);
         System.out.println("Mssg Payload Size: " + data.payloadSize);
      }

      if (data.messageID == 4)
      {
         System.out.println("Description Message");
         data.numberOfDataSets = buf.getInt();

         System.out.println("# of datasets: " + data.numberOfDataSets);

         for (int i = 0; i <= data.numberOfDataSets; i++)
         {
            System.out.println("Desc. Type: " + buf.getInt());
         }

         data.type = buf.getInt();

         System.out.println("Type: " + data.type);

      }

      if (data.messageID == 7)
      {
         data.frameNumber = buf.getInt();
         data.numberOfDataSets = buf.getInt();

         if (DEBUG)
         {
            System.out.println("Message type: FrameData Message");
            System.out.println("Frame #: " + data.frameNumber);
            System.out.println("# of datasets: " + data.numberOfDataSets);
            System.out.println("# of unidentified markers: " + data.numberOfUnidentifiedMarkers);
            System.out.println("# of rigid bodies: " + data.numberOfRigidBodies);
         }

         for (int i = 0; i < data.numberOfDataSets; i++)
         {
         }

         data.numberOfUnidentifiedMarkers = buf.getInt();

         for (int i = 0; i < data.numberOfUnidentifiedMarkers; i++)
         {
            data.uMarkerX = buf.getFloat();
            data.uMarkerY = buf.getFloat();
            data.uMarkerZ = buf.getFloat();
         }

         data.numberOfRigidBodies = buf.getInt();

         for (int i = 0; i < data.numberOfRigidBodies; i++)
         {
            data.rigidBodyId = buf.getInt();
            data.posX = buf.getFloat();
            data.posY = buf.getFloat();
            data.posZ = buf.getFloat();
            data.rotX = buf.getFloat();
            data.rotY = buf.getFloat();
            data.rotZ = buf.getFloat();
            data.rotW = buf.getFloat();
            data.nRigidMarkers = buf.getInt();

            data.markerPosition = new Vector3D[data.nRigidMarkers];


            for (int j = 0; j < data.nRigidMarkers; j++)
            {
               data.markerPosition[j] = new Vector3D(buf.getFloat(), buf.getFloat(), buf.getFloat());
            }

            data.markerIds = new int[data.nRigidMarkers];

            for (int j = 0; j < data.nRigidMarkers; j++)
            {
               data.markerIds[j] = buf.getInt();
            }

            data.markerSizes = new float[data.nRigidMarkers];

            for (int j = 0; j < data.nRigidMarkers; j++)
            {
               data.markerSizes[j] = buf.getFloat();
            }

            short params = 10;
            params = buf.getShort();

            data.isTracked = params != 0;

            data.meanMarkerError = buf.getFloat();

            ArrayList<MocapMarker> listfOfMarkersForThisRB = new ArrayList<>();

            for (int j = 0; j < data.nRigidMarkers; j++)
            {
               MocapMarker mocapMarker = new MocapMarker(data.markerIds[j], data.markerPosition[j], data.markerSizes[j]);
               listfOfMarkersForThisRB.add(mocapMarker);
            }

            rigidBody = new MocapRigidBody(data.rigidBodyId, new Vector3D(data.posX, data.posY, data.posZ),
                                           new Quaternion(data.rotX, data.rotY, data.rotZ, data.rotW), listfOfMarkersForThisRB, data.isTracked);

            listfOfRigidbodies.add(rigidBody);

            if (DEBUG)
            {
               System.out.println("\nInformation for rigid body " + i + ": ");
               rigidBody.toString();

//             System.out.println("Tracking: " + (params & 0x01));

            }
         }

         data.numberOfSkeletons = buf.getInt();
         data.numberOfLabeledMarkers = buf.getInt();
         data.latency = buf.getFloat();

         if (DEBUG)
         {
            System.out.println("# of Skeletons: " + data.numberOfSkeletons);
            System.out.println("Labeled Markers: " + data.numberOfLabeledMarkers);
            System.out.println("Latency: " + data.latency);

            System.out.println("<<<<<<<<<<<<<<<<<<<<<<END MESSAGE");
         }
      }

      return listfOfRigidbodies;
   }
}
