package us.ihmc.atlas.multisenseMocapExperiments;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Matrix3d;
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

import com.badlogic.gdx.physics.bullet.softbody.btSoftBody.Pose;
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

   private static void getPitchFromMarkers(Point3d[] points)
   {
      Point3d p0 = points[0];
      Point3d p1 = points[1];
      Point3d p2 = points[2];
//      System.out.println(p0);
//      System.out.println(p1);
//      System.out.println(p2);
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
//      coordinate level
//      Point3d p1 = new Point3d(0.007940,0.055415,0.292275);
//      Point3d p2 = new Point3d(0.009908,0.055159,-0.007941);
//      Point3d p3 = new Point3d(-0.189175, 0.055152, -0.009826);
      
//      coordinate pitched, 33 deg
      Point3d p1 = new Point3d(0.043280,0.045145,0.294899);
      Point3d p2 = new Point3d(0.043760,0.047519,-0.005182);
      Point3d p3 = new Point3d(-0.122070,-0.064075,-0.006423   );
      
      
//      Vector3d v1p2p1 = new Vector3d(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
//      Vector3d v2p2p3 = new Vector3d(p2.x - p3.x, p2.y - p3.y, p2.z - p3.z);
//      Vector3d crossProduct = new Vector3d();
//      
//      crossProduct.cross(v1p2p1, v2p2p3);
//      crossProduct.normalize();
      
      
      Point3d[] points = new Point3d[3];
      points[0] = p1;
      points[1] = p2;
      points[2] = p3;
      
      getPitchFromMarkers(points);
      
      boolean headless = false;
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasTarget.HEAD_ON_A_STICK, headless);

      new AtlasMinimalMultisenseMocapNetworkProcessor(robotModel);
   }
}
