package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Quat4d;
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
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class IHMCMOCAPLocalizationModule implements MocapRigidbodiesListener, PacketConsumer<RobotConfigurationData>
{
   private static final double MOCAP_SERVER_DT = 0.002;
   private static final int PELVIS_ID = 1;

   private RobotConfigurationData latestRobotConfigurationData = null;
   private final MocapToPelvisFrameConverter mocapToPelvisFrameConverter = new MocapToPelvisFrameConverter();
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE, new IHMCCommunicationKryoNetClassList());
   private final YoVariableRegistry yoVariableRegistry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final FullHumanoidRobotModel fullRobotModel;
   private final RigidBodyTransform pelvisTransform = new RigidBodyTransform();
   
   private final Vector3f pelvisTranslation = new Vector3f();
   private final Quat4f pelvisOrientation = new Quat4f(0.0f, 0.0f, 0.0f, 1.0f);
   private final ReferenceFrame pelvisFrame = new ReferenceFrame("pelvisFrame", ReferenceFrame.getWorldFrame())
         {
            private static final long serialVersionUID = 116774591450076114L;

            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.setTranslation(pelvisTranslation);
               transformToParent.setRotation(pelvisOrientation);
            }      
         };
   
   private final DoubleYoVariable markersPositionX = new DoubleYoVariable("markersPositionX", yoVariableRegistry);
   private final DoubleYoVariable markersPositionY = new DoubleYoVariable("markersPositionY", yoVariableRegistry);
   private final DoubleYoVariable markersPositionZ = new DoubleYoVariable("markersPositionZ", yoVariableRegistry);
   
   private final DoubleYoVariable markersYaw = new DoubleYoVariable("markersYaw", yoVariableRegistry);
   private final DoubleYoVariable markersPitch = new DoubleYoVariable("markersPitch", yoVariableRegistry);
   private final DoubleYoVariable markersRoll = new DoubleYoVariable("markersRoll", yoVariableRegistry);

   private final DoubleYoVariable computedPelvisPositionX = new DoubleYoVariable("computedPelvisPositionX", yoVariableRegistry);
   private final DoubleYoVariable computedPelvisPositionY = new DoubleYoVariable("computedPelvisPositionY", yoVariableRegistry);
   private final DoubleYoVariable computedPelvisPositionZ = new DoubleYoVariable("computedPelvisPositionZ", yoVariableRegistry);

   private final DoubleYoVariable computedPelvisYaw = new DoubleYoVariable("computedPelvisYaw", yoVariableRegistry);
   private final DoubleYoVariable computedPelvisPitch = new DoubleYoVariable("computedPelvisPitch", yoVariableRegistry);
   private final DoubleYoVariable computedPelvisRoll = new DoubleYoVariable( "computedPelvisRoll", yoVariableRegistry);
   
   private final FramePoint marker1Point = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint marker2Point = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint marker3Point = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint marker4Point = new FramePoint(ReferenceFrame.getWorldFrame());
   
   private final DoubleYoVariable pelvisTranslationX = new DoubleYoVariable("pelvisTranslationX", yoVariableRegistry);
   private final DoubleYoVariable pelvisTranslationY = new DoubleYoVariable("pelvisTranslationY", yoVariableRegistry);
   private final DoubleYoVariable pelvisTranslationZ = new DoubleYoVariable("pelvisTranslationZ", yoVariableRegistry);
   private final DoubleYoVariable pelvisYaw = new DoubleYoVariable("pelvisYaw", yoVariableRegistry);
   private final DoubleYoVariable pelvisPitch = new DoubleYoVariable("pelvisPitch", yoVariableRegistry);
   private final DoubleYoVariable pelvisRoll = new DoubleYoVariable("pelvisRoll", yoVariableRegistry);
   
   private final DoubleYoVariable marker1PositionX = new DoubleYoVariable("marker1PositionX", yoVariableRegistry);
   private final DoubleYoVariable marker1PositionY = new DoubleYoVariable("marker1PositionY", yoVariableRegistry);
   private final DoubleYoVariable marker1PositionZ = new DoubleYoVariable("marker1PositionZ", yoVariableRegistry);

   private final DoubleYoVariable marker2PositionX = new DoubleYoVariable("marker2PositionX", yoVariableRegistry);
   private final DoubleYoVariable marker2PositionY = new DoubleYoVariable("marker2PositionY", yoVariableRegistry);
   private final DoubleYoVariable marker2PositionZ = new DoubleYoVariable("marker2PositionZ", yoVariableRegistry);

   private final DoubleYoVariable marker3PositionX = new DoubleYoVariable("marker3PositionX", yoVariableRegistry);
   private final DoubleYoVariable marker3PositionY = new DoubleYoVariable("marker3PositionY", yoVariableRegistry);
   private final DoubleYoVariable marker3PositionZ = new DoubleYoVariable("marker3PositionZ", yoVariableRegistry);

   private final DoubleYoVariable marker4PositionX = new DoubleYoVariable("marker4PositionX", yoVariableRegistry);
   private final DoubleYoVariable marker4PositionY = new DoubleYoVariable("marker4PositionY", yoVariableRegistry);
   private final DoubleYoVariable marker4PositionZ = new DoubleYoVariable("marker4PositionZ", yoVariableRegistry);

   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel) 
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
      
      packetCommunicator.attachListener(RobotConfigurationData.class, this);

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

      if(!mocapToPelvisFrameConverter.isInitialized() && latestRobotConfigurationData != null)
      {
         ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
         MocapRigidBody pelvisRigidBody = getPelvisRigidBody(listOfRigidbodies);
         mocapToPelvisFrameConverter.initialize(pelvisFrame, pelvisRigidBody);
      }
      else if(latestRobotConfigurationData == null)
      {
         PrintTools.info("Waiting for robot configuration data");
         return;
      }

      MocapRigidBody pelvisRigidBody = getPelvisRigidBody(listOfRigidbodies);
      sendPelvisTransformToController(pelvisRigidBody);
      setMarkersYoVariables(pelvisRigidBody);
      
      yoVariableServer.update(System.currentTimeMillis());
   }

   private MocapRigidBody getPelvisRigidBody(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody rigidBody = listOfRigidbodies.get(i);
         if(rigidBody.getId() == PELVIS_ID)
            return rigidBody;
      }

      return null;
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

   private void setPelvisYoVariables(RigidBodyTransform pelvisTransform)
   {
      Vector3d pelvisTranslation = new Vector3d();
      double[] yawPitchRoll = new double[3];

      pelvisTransform.getTranslation(pelvisTranslation);

      Quat4d pelvisRotation = new Quat4d();
      pelvisTransform.getRotation(pelvisRotation);
      RotationTools.convertQuaternionToYawPitchRoll(pelvisRotation, yawPitchRoll);

      computedPelvisPositionX.set(pelvisTranslation.getX());
      computedPelvisPositionY.set(pelvisTranslation.getY());
      computedPelvisPositionZ.set(pelvisTranslation.getZ());

      computedPelvisYaw.set(yawPitchRoll[0]);
      computedPelvisPitch.set(yawPitchRoll[1]);
      computedPelvisRoll.set(yawPitchRoll[2]);
   }

   private void sendPelvisTransformToController(MocapRigidBody pelvisRigidBody)
   {
      mocapToPelvisFrameConverter.computePelvisToWorldTransform(pelvisRigidBody, pelvisTransform);
      setPelvisYoVariables(pelvisTransform);

      if(latestRobotConfigurationData != null)
      {
         TimeStampedTransform3D timestampedTransform = new TimeStampedTransform3D(pelvisTransform, latestRobotConfigurationData.getTimestamp());
         StampedPosePacket stampedPosePacket = new StampedPosePacket(Integer.toString(PELVIS_ID), timestampedTransform, 1.0);
         
         stampedPosePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
         packetCommunicator.send(stampedPosePacket);      
      }
      else
      {
         System.err.println("Haven't received timestamp from controller, ignoring mocap data");
      }
   }
   
   private void updateMarkerPoints()
   {
      pelvisFrame.update();
      
      marker1Point.setToZero(pelvisFrame);
      marker2Point.setToZero(pelvisFrame);
      marker3Point.setToZero(pelvisFrame);
      marker4Point.setToZero(pelvisFrame);
      
      marker1Point.set(mocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(1));
      marker2Point.set(mocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(2));
      marker3Point.set(mocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(3));
      marker4Point.set(mocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(4));
      
      marker1Point.changeFrame(ReferenceFrame.getWorldFrame());
      marker2Point.changeFrame(ReferenceFrame.getWorldFrame());
      marker3Point.changeFrame(ReferenceFrame.getWorldFrame());
      marker4Point.changeFrame(ReferenceFrame.getWorldFrame());
      
      marker1PositionX.set(marker1Point.getX());
      marker1PositionY.set(marker1Point.getY());
      marker1PositionZ.set(marker1Point.getZ());

      marker2PositionX.set(marker2Point.getX());
      marker2PositionY.set(marker2Point.getY());
      marker2PositionZ.set(marker2Point.getZ());

      marker3PositionX.set(marker3Point.getX());
      marker3PositionY.set(marker3Point.getY());
      marker3PositionZ.set(marker3Point.getZ());

      marker4PositionX.set(marker4Point.getX());
      marker4PositionY.set(marker4Point.getY());
      marker4PositionZ.set(marker4Point.getZ());
      
      pelvisTranslationX.set(pelvisTranslation.getX());
      pelvisTranslationY.set(pelvisTranslation.getY());
      pelvisTranslationZ.set(pelvisTranslation.getZ());
      
      double[] pelvisYPR = new double[3];
      RotationTools.convertQuaternionToYawPitchRoll(pelvisOrientation, pelvisYPR);
      
      pelvisYaw.set(pelvisYPR[0]);
      pelvisPitch.set(pelvisYPR[1]);
      pelvisRoll.set(pelvisYPR[2]);
   }
   
   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {      
      latestRobotConfigurationData = packet;
      
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

      pelvisTranslation.set(packet.getPelvisTranslation());
      pelvisOrientation.set(packet.getPelvisOrientation());

      rootJoint.setPosition(pelvisTranslation.getX(), pelvisTranslation.getY(), pelvisTranslation.getZ());
      rootJoint.setRotation(pelvisOrientation.getX(), pelvisOrientation.getY(), pelvisOrientation.getZ(), pelvisOrientation.getW());
      
      rootJoint.getPredecessor().updateFramesRecursively();   
      updateMarkerPoints();
      yoVariableServer.update(System.currentTimeMillis());
   }   
}
