package us.ihmc.atlas.multisenseMocapExperiments;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import optitrack.MocapDataClient;
import optitrack.MocapMarker;
import optitrack.MocapRigidBody;
import optitrack.MocapRigidbodiesListener;
import sensor_msgs.PointCloud2;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

import com.martiansoftware.jsap.JSAPException;

public class AtlasMinimalMultisenseMocapNetworkProcessor  extends RosPointCloudSubscriber implements MocapRigidbodiesListener
{
   private static final NetClassList NETCLASSLIST = new IHMCCommunicationKryoNetClassList();

   private static final int MULTISENSE_MOCAP_ID = 0;
   private static final int BOX_MOCAP_ID = 2;

   private final PacketCommunicator uiPacketServer = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, NETCLASSLIST);
   private final RigidBodyTransform orientationTransformFromLeftOpticalFrameToZUp = new RigidBodyTransform();
   private final MocapDataClient mocapDataClient;
   private final AtomicReference<RigidBodyTransform> headPoseInZUp = new  AtomicReference<RigidBodyTransform>(new RigidBodyTransform());
   
   public AtlasMinimalMultisenseMocapNetworkProcessor(DRCRobotModel robotModel) throws IOException
   {
      URI rosMasterURI = NetworkParameters.getROSURI();
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "atlas/AtlasMinimalMultisenseMocapNetworkProcessor", true);
      rosMainNode.attachSubscriber("/multisense/lidar_points2", this);
      rosMainNode.execute();
      uiPacketServer.connect();
      
      mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
      System.out.println("running minimal experiment");
   }
   
   RigidBodyTransform rpyCalibrationOffset = new RigidBodyTransform();
   
   /**
    * received a point cloud from the multisense
    */
   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
      Point3d[] points = pointCloudData.getPoints();
      orientationTransformFromLeftOpticalFrameToZUp.setEuler(-Math.PI/2, 0.0, -Math.PI/2);
      rpyCalibrationOffset.setEuler(Math.toRadians(1.2), Math.toRadians(0.5), Math.toRadians(2));
      
      for(int i = 0; i < points.length; i++)
      {
         orientationTransformFromLeftOpticalFrameToZUp.transform(points[i]);
         rpyCalibrationOffset.transform(points[i]);
         headPoseInZUp.get().transform(points[i]);
      }
      
      PointCloudWorldPacket pointCloudPacket = new PointCloudWorldPacket();
      pointCloudPacket.setDecayingWorldScan(points);
      uiPacketServer.send(pointCloudPacket);
   }
   
   /**
    * received an update from mocap
    */
   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody mocapObject = listOfRigidbodies.get(i);
         int id = mocapObject.getId();

         if (id == MULTISENSE_MOCAP_ID)
         {
            Point3d[] points = new Point3d[3];
            ArrayList<MocapMarker> listOfAssociatedMarkers = mocapObject.getListOfAssociatedMarkers();
            for (int markerIndex = 0; markerIndex < listOfAssociatedMarkers.size(); markerIndex++)
            {
               MocapMarker mocapMarker = listOfAssociatedMarkers.get(markerIndex);
               points[markerIndex] = new Point3d(mocapMarker.getPosition());
            }
            RigidBodyTransform pose = new RigidBodyTransform();
            mocapObject.getPoseInZUp(pose);
            headPoseInZUp.set(pose);
         }
      }
   }

   /**
    * try to calculate the pitch using the cross product of the markers of a 
    * triangle shaped forward facing mocap rigid bopy
    * @param points
    */
   private static void getPitchFromMarkers(Point3d[] points)
   {
      Point3d p0 = points[0];
      Point3d p1 = points[1];
      Point3d p2 = points[2];
      
      Vector3d v1p2p1 = new Vector3d(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
      Vector3d v2p2p3 = new Vector3d(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);

      Vector3d crossProduct = new Vector3d();
      crossProduct.cross(v1p2p1, v2p2p3);
      crossProduct.normalize();
      
      Vector3d xForward = new Vector3d(1, 0, 0);
      
      Quat4d rotation = getRotationQuat(xForward, crossProduct);
      double[] yawPitchRollToPack = new double[3];
      RotationFunctions.setYawPitchRollBasedOnQuaternion(yawPitchRollToPack , rotation);
      System.out.println(Math.toDegrees(yawPitchRollToPack[1]));
   }
   
   /**
    * get quaternion representing the rotation difference between two vectors
    * @param from starting vector
    * @param to ending vector
    * @return quaternion representing the rotation difference
    */
   public static Quat4d getRotationQuat(Vector3d from, Vector3d to)
   {
      Quat4d result = new Quat4d();

      Vector3d H = new Vector3d(from);
      H.add(to);
      H.normalize();

      result.w = from.dot(H);
      result.x = from.y * H.z - from.z * H.y;
      result.y = from.z * H.x - from.x * H.z;
      result.z = from.x * H.y - from.y * H.x;
      return result;
   }
   
   public static void main(String[] args) throws JSAPException, IOException
   {
      boolean headless = false;
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasTarget.HEAD_ON_A_STICK, headless);

      new AtlasMinimalMultisenseMocapNetworkProcessor(robotModel);
   }
}
