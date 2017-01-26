package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class IHMCMOCAPLocalizationModule implements MocapRigidbodiesListener, PacketConsumer<RobotConfigurationData>
{
   private static final double MOCAP_SERVER_DT = 0.002;
   private static final int PELVIS_ID = 1;
   
   private boolean timestampHasBeenSet = false;
   private long latestRobotTimeStamp = Long.MIN_VALUE;
   
   private final MocapToPelvisFrameConverter mocapToPelvisFrameConverter;
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE, new IHMCCommunicationKryoNetClassList());
   private final YoVariableRegistry yoVariableRegistry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final FullHumanoidRobotModel fullRobotModel;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   
   private final DoubleYoVariable markersPositionX = new DoubleYoVariable("markersPositionX", yoVariableRegistry);
   private final DoubleYoVariable markersPositionY = new DoubleYoVariable("markersPositionY", yoVariableRegistry);
   private final DoubleYoVariable markersPositionZ = new DoubleYoVariable("markersPositionZ", yoVariableRegistry);
   
   private final DoubleYoVariable markersYaw = new DoubleYoVariable("markersYaw", yoVariableRegistry);
   private final DoubleYoVariable markersPitch = new DoubleYoVariable("markersPitch", yoVariableRegistry);
   private final DoubleYoVariable markersRoll = new DoubleYoVariable("markersRoll", yoVariableRegistry);
   
   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel) 
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
      
      packetCommunicator.attachListener(RobotConfigurationData.class, this);
      
      mocapToPelvisFrameConverter = new MocapToPelvisFrameConverter(drcRobotModel, packetCommunicator);
      robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      
      ppsTimestampOffsetProvider = drcRobotModel.getPPSTimestampOffsetProvider();
      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("MocapModuleScheduler");
      LogModelProvider logModelProvider = drcRobotModel.getLogModelProvider();
      
      yoVariableServer = new YoVariableServer(getClass(), scheduler, logModelProvider, drcRobotModel.getLogSettings(), MOCAP_SERVER_DT);
      fullRobotModel = drcRobotModel.createFullRobotModel();
      yoVariableServer.setMainRegistry(yoVariableRegistry, fullRobotModel, null);
      
      PrintTools.info("Starting server");
      yoVariableServer.start();
      
      try
      {
         packetCommunicator.connect();         
      }
      catch(IOException e)
      {
         e.printStackTrace();
      }
   }
   
   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      if(! packetCommunicator.isConnected())
      {
         PrintTools.info("Packet communicator isn't registered, ignoring MOCAP data");
         return;
      }
      
      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody mocapRigidBody = listOfRigidbodies.get(i);
         
         if(mocapRigidBody.getId() == PELVIS_ID)
         {
            RigidBodyTransform pelvisTransform = new RigidBodyTransform();
            // mocapToPelvisFrameConverter.convertMocapPoseToRobotFrame(mocapRigidBody);
            mocapRigidBody.packPose(pelvisTransform);
            sendPelvisTransformToController(pelvisTransform);   
            setMarkersYoVariables(mocapRigidBody);
         }
      }
      
      yoVariableServer.update(System.currentTimeMillis());
   }

   private void setMarkersYoVariables(MocapRigidBody mocapRigidBody)
   {
      markersPositionX.set(mocapRigidBody.xPosition);
      markersPositionY.set(mocapRigidBody.yPosition);
      markersPositionZ.set(mocapRigidBody.zPosition);
      
      double[] yawPitchRoll = new double[3];
      RotationTools.convertQuaternionToYawPitchRoll(mocapRigidBody.getOrientation(), yawPitchRoll);
      
      markersYaw.set(yawPitchRoll[0]);      
      markersPitch.set(yawPitchRoll[1]);      
      markersRoll.set(yawPitchRoll[2]);      
   }

   private void sendPelvisTransformToController(RigidBodyTransform pelvisTransform)
   {      
      if(timestampHasBeenSet)
      {
         TimeStampedTransform3D timestampedTransform = new TimeStampedTransform3D(pelvisTransform, latestRobotTimeStamp);      
         StampedPosePacket stampedPosePacket = new StampedPosePacket(Integer.toString(PELVIS_ID), timestampedTransform, 1.0);
         
         stampedPosePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
         packetCommunicator.send(stampedPosePacket);      
      }
      else
      {
         System.err.println("Haven't received timestamp from controller, ignoring mocap data");
      }
   }
   
   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      timestampHasBeenSet = true;
      latestRobotTimeStamp = packet.lastReceivedPacketRobotTimestamp;
      
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();

      float[] newJointAngles = packet.getJointAngles();
      float[] newJointVelocities = packet.getJointAngles();
      float[] newJointTorques = packet.getJointTorques();
      OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();
      
      for (int i = 0; i < newJointAngles.length; i++)
      {
         oneDoFJoints[i].setQ(newJointAngles[i]);
         oneDoFJoints[i].setQd(newJointVelocities[i]);
         oneDoFJoints[i].setTau(newJointTorques[i]);
      }

      Vector3f translation = packet.getPelvisTranslation();
      Quat4f orientation = packet.getPelvisOrientation();

      rootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
      rootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getW());
      
      rootJoint.getPredecessor().updateFramesRecursively();      
   }
}
