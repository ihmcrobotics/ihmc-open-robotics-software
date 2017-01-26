package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class IHMCMOCAPLocalizationModule implements MocapRigidbodiesListener
{
   private static final double MOCAP_SERVER_DT = 0.002;
   private static final int PELVIS_ID = 1;
   
   private final MocapToPelvisFrameConverter mocapToPelvisFrameConverter;
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE, new IHMCCommunicationKryoNetClassList());
   private final YoVariableRegistry yoVariableRegistry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final FullHumanoidRobotModel fullRobotModel;
   
   private final DoubleYoVariable pelvisPositionX = new DoubleYoVariable("pelvisPositionX", yoVariableRegistry);
   private final DoubleYoVariable pelvisPositionY = new DoubleYoVariable("pelvisPositionY", yoVariableRegistry);
   private final DoubleYoVariable pelvisPositionZ = new DoubleYoVariable("pelvisPositionZ", yoVariableRegistry);
   
   private final Vector3d tempPosition = new Vector3d();
   private final Matrix3d tempOrientation = new Matrix3d();
   
   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel) 
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);

      mocapToPelvisFrameConverter = new MocapToPelvisFrameConverter(drcRobotModel, packetCommunicator);
      
      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("MocapModuleScheduler");
      LogModelProvider logModelProvider = drcRobotModel.getLogModelProvider();
      
      yoVariableServer = new YoVariableServer(getClass(), scheduler, logModelProvider, LogSettings.ATLAS_IAN, MOCAP_SERVER_DT);
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
      
      PrintTools.info("\nReceived rigid bodies:");

      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody mocapRigidBody = listOfRigidbodies.get(i);
         
         if(mocapRigidBody.getId() == PELVIS_ID)
         {
            RigidBodyTransform pelvisTransform = new RigidBodyTransform();
            // mocapToPelvisFrameConverter.convertMocapPoseToRobotFrame(mocapRigidBody);
            mocapRigidBody.packPose(pelvisTransform);
            sendPelvisTransformToController(pelvisTransform);            
            PrintTools.info("Transform : \n" + pelvisTransform);
         }
      }
      
      yoVariableServer.update(System.currentTimeMillis());
   }

   private void sendPelvisTransformToController(RigidBodyTransform pelvisTransform)
   {      
      TimeStampedTransform3D timestampedTransform = createTimestampedTransform(pelvisTransform);
      StampedPosePacket stampedPosePacket = new StampedPosePacket(Integer.toString(PELVIS_ID), timestampedTransform, 1.0);
      
      stampedPosePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
      packetCommunicator.send(stampedPosePacket);
      
      pelvisTransform.getTranslation(tempPosition);
      pelvisTransform.getRotation(tempOrientation);
            
      fullRobotModel.getRootJoint().setPosition(tempPosition);
      fullRobotModel.getRootJoint().setRotation(tempOrientation);
      
      pelvisPositionX.set(tempPosition.getX());
      pelvisPositionY.set(tempPosition.getY());
      pelvisPositionZ.set(tempPosition.getZ());
   }

   private TimeStampedTransform3D createTimestampedTransform(RigidBodyTransform rigidBodyTransform)
   {
      return new TimeStampedTransform3D(rigidBodyTransform, System.currentTimeMillis());
   }
}
