package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.io.IOException;
import java.util.ArrayList;

import gnu.trove.list.array.TFloatArrayList;
import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class IHMCMOCAPLocalizationModule implements MocapRigidbodiesListener, PacketConsumer<RobotConfigurationData>
{
   private static final double MOCAP_SERVER_DT = 0.002;
   private static final int PELVIS_ID = 1;

   private RobotConfigurationData latestRobotConfigurationData = null;
   private final MocapToPelvisFrameConverter mocapToPelvisFrameConverter = new MocapToPelvisFrameConverter();
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE,
                                                                                                                 new IHMCCommunicationKryoNetClassList());
   
   PacketCommunicator mocapVizPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.MOCAP_MODULE_VIZ, new IHMCCommunicationKryoNetClassList());
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final FullHumanoidRobotModel fullRobotModel;
   private final MocapPlanarRegionsListManager planarRegionsListManager;
   private final WalkingStatusManager walkingStatusManager = new WalkingStatusManager();

   private final Vector3D32 pelvisTranslationFromRobotConfigurationData = new Vector3D32();
   private final Quaternion32 pelvisOrientationFromRobotConfigurationData = new Quaternion32(0.0f, 0.0f, 0.0f, 1.0f);
   private final ReferenceFrame pelvisFrameFromRobotConfigurationDataPacket = new ReferenceFrame("pelvisFrameFromRobotConfigurationDataPacket", ReferenceFrame.getWorldFrame())
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setTranslation(pelvisTranslationFromRobotConfigurationData);
         transformToParent.setRotation(pelvisOrientationFromRobotConfigurationData);
      }
   };
   
   private RigidBodyTransform pelvisToWorldFromMocapData = new RigidBodyTransform();
   private final ReferenceFrame pelvisFrameFromMocap = new ReferenceFrame("pelvisFrameFromMocap", ReferenceFrame.getWorldFrame())
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(pelvisToWorldFromMocapData);
      }
   };
   
   private final YoDouble computedPelvisPositionX = new YoDouble("computedPelvisPositionX", registry);
   private final YoDouble computedPelvisPositionY = new YoDouble("computedPelvisPositionY", registry);
   private final YoDouble computedPelvisPositionZ = new YoDouble("computedPelvisPositionZ", registry);

   private final YoDouble computedPelvisYaw = new YoDouble("computedPelvisYaw", registry);
   private final YoDouble computedPelvisPitch = new YoDouble("computedPelvisPitch", registry);
   private final YoDouble computedPelvisRoll = new YoDouble("computedPelvisRoll", registry);
   
   private final YoDouble mocapWorldToRobotWorldTransformX = new YoDouble("mocapWorldToRobotWorldTransformX", registry);
   private final YoDouble mocapWorldToRobotWorldTransformY = new YoDouble("mocapWorldToRobotWorldTransformY", registry);
   private final YoDouble mocapWorldToRobotWorldTransformZ = new YoDouble("mocapWorldToRobotWorldTransformZ", registry);
   private final YoDouble mocapWorldToRobotWorldTransformYaw = new YoDouble("mocapWorldToRobotWorldTransformYaw", registry);
   private final YoDouble mocapWorldToRobotWorldTransformPitch = new YoDouble("mocapWorldToRobotWorldTransformPitch", registry);
   private final YoDouble mocapWorldToRobotWorldTransformRoll = new YoDouble("mocapWorldToRobotWorldTransformRoll", registry);

   private final YoBoolean requestReInitialization = new YoBoolean("requestReInitialization", registry);
   private final YoBoolean requestFootsteps = new YoBoolean("requestFootsteps", registry);
   private final YoBoolean walkingAround = new YoBoolean("walkingAround", registry);
      
   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel, MocapPlanarRegionsListManager planarRegionsListManager)
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);

      packetCommunicator.attachListener(RobotConfigurationData.class, this);
      packetCommunicator.attachListener(FootstepStatusMessage.class, walkingStatusManager);

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
      LogModelProvider logModelProvider = drcRobotModel.getLogModelProvider();
      
      this.planarRegionsListManager = planarRegionsListManager;

      yoVariableServer = new YoVariableServer(getClass(), scheduler, logModelProvider, drcRobotModel.getLogSettings(), MOCAP_SERVER_DT);
      fullRobotModel = drcRobotModel.createFullRobotModel();
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), null);

      PrintTools.info("Starting server");
      yoVariableServer.start();
      
      try
      {
         packetCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      if (!packetCommunicator.isConnected())
      {
         PrintTools.info("Packet communicator isn't registered, ignoring MOCAP data");
         return;
      }

      if (!mocapToPelvisFrameConverter.isInitialized() && latestRobotConfigurationData != null)
      {
         initializeMocapFrameConverter(listOfRigidbodies);
      }
      else if (latestRobotConfigurationData == null)
      {
         PrintTools.info("Waiting for robot configuration data");
         return;
      }
            
      if(requestReInitialization.getBooleanValue())
      {
         initializeMocapFrameConverter(listOfRigidbodies);
         requestReInitialization.set(false);
      }

      MocapRigidBody pelvisRigidBody = getPelvisRigidBody(listOfRigidbodies);
      sendPelvisTransformToController(pelvisRigidBody);
      
      if (requestFootsteps.getBooleanValue())
      {
         planarRegionsListManager.savePlanarRegionsAfterWalking();
         startWalking();
         walkingAround.set(true);
         requestFootsteps.set(false);
      }
      
      if(walkingAround.getBooleanValue() && walkingStatusManager.doneWalking())
      {
         planarRegionsListManager.savePlanarRegionsAfterWalking();
         walkingAround.set(false);
      }
     
      yoVariableServer.update(System.currentTimeMillis());
   }
   
   private void initializeMocapFrameConverter(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      pelvisFrameFromRobotConfigurationDataPacket.update();
      MocapRigidBody pelvisRigidBody = getPelvisRigidBody(listOfRigidbodies);
      mocapToPelvisFrameConverter.initialize(pelvisFrameFromRobotConfigurationDataPacket, pelvisRigidBody);
      
   }

   private MocapRigidBody getPelvisRigidBody(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody rigidBody = listOfRigidbodies.get(i);
         if (rigidBody.getId() == PELVIS_ID)
            return rigidBody;
      }

      return null;
   }
   
   private void setPelvisYoVariables(RigidBodyTransform pelvisTransform)
   {
      Vector3D pelvisTranslation = new Vector3D();
      double[] yawPitchRoll = new double[3];

      pelvisTransform.getTranslation(pelvisTranslation);

      Quaternion pelvisRotation = new Quaternion();
      pelvisTransform.getRotation(pelvisRotation);
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(pelvisRotation, yawPitchRoll);

      computedPelvisPositionX.set(pelvisTranslation.getX());
      computedPelvisPositionY.set(pelvisTranslation.getY());
      computedPelvisPositionZ.set(pelvisTranslation.getZ());

      computedPelvisYaw.set(yawPitchRoll[0]);
      computedPelvisPitch.set(yawPitchRoll[1]);
      computedPelvisRoll.set(yawPitchRoll[2]);
   }

   private void sendPelvisTransformToController(MocapRigidBody pelvisRigidBody)
   {
      mocapToPelvisFrameConverter.computePelvisToWorldTransform(pelvisRigidBody, pelvisToWorldFromMocapData);
      setPelvisYoVariables(pelvisToWorldFromMocapData);

      //      if(latestRobotConfigurationData != null)
      //      {
      //         TimeStampedTransform3D timestampedTransform = new TimeStampedTransform3D(pelvisToWorldTransform, latestRobotConfigurationData.getTimestamp());
      //         StampedPosePacket stampedPosePacket = new StampedPosePacket(Integer.toString(PELVIS_ID), timestampedTransform, 1.0);
      //         
      //         stampedPosePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
      //         packetCommunicator.send(stampedPosePacket);      
      //      }
      //      else
      //      {
      //         System.err.println("Haven't received timestamp from controller, ignoring mocap data");
      //      }
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      latestRobotConfigurationData = packet;

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();

      TFloatArrayList newJointAngles = packet.getJointAngles();
      TFloatArrayList newJointVelocities = packet.getJointAngles();
      TFloatArrayList newJointTorques = packet.getJointTorques();
      OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(newJointVelocities.get(i));
         oneDoFJoints[i].setTau(newJointTorques.get(i));
      }

      pelvisTranslationFromRobotConfigurationData.set(packet.getPelvisTranslation());
      pelvisOrientationFromRobotConfigurationData.set(packet.getPelvisOrientation());

      rootJoint.setPosition(pelvisTranslationFromRobotConfigurationData.getX(), pelvisTranslationFromRobotConfigurationData.getY(), pelvisTranslationFromRobotConfigurationData.getZ());
      rootJoint.setRotation(pelvisOrientationFromRobotConfigurationData.getX(), pelvisOrientationFromRobotConfigurationData.getY(), pelvisOrientationFromRobotConfigurationData.getZ(), pelvisOrientationFromRobotConfigurationData.getS());
      
      computeDriftTransform();

      rootJoint.getPredecessor().updateFramesRecursively();
      yoVariableServer.update(System.currentTimeMillis());
   }

   private void computeDriftTransform()
   {
      RigidBodyTransform driftTransform = new RigidBodyTransform();
      pelvisFrameFromMocap.update();
      pelvisFrameFromRobotConfigurationDataPacket.update();
      pelvisFrameFromMocap.getTransformToDesiredFrame(driftTransform, pelvisFrameFromRobotConfigurationDataPacket);
      
      Vector3D driftTranslation = new Vector3D();
      driftTransform.getTranslation(driftTranslation);
      
      Quaternion driftRotation = new Quaternion();
      driftTransform.getRotation(driftRotation);
      double[] driftRotationYPR = new double[3];
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(driftRotation, driftRotationYPR);
      
      mocapWorldToRobotWorldTransformX.set(driftTranslation.getX());
      mocapWorldToRobotWorldTransformY.set(driftTranslation.getY());
      mocapWorldToRobotWorldTransformZ.set(driftTranslation.getZ());
      
      mocapWorldToRobotWorldTransformYaw.set(driftRotationYPR[0]);
      mocapWorldToRobotWorldTransformPitch.set(driftRotationYPR[1]);
      mocapWorldToRobotWorldTransformRoll.set(driftRotationYPR[2]);      
   }
   
   public void startWalking()
   {
      ArrayList<FootstepDataMessage> listOfStepsForward = new ArrayList<>(8);
      ArrayList<FootstepDataMessage> listOfStepsBackward = new ArrayList<>(8);
      RobotSide robotSide = RobotSide.LEFT;
      boolean isDirectionForward = true;

      listOfStepsForward = createFootstepList(robotSide, isDirectionForward);
      listOfStepsBackward = createFootstepList(robotSide, !isDirectionForward);
      
      ArrayList<FootstepDataMessage> overallListOfSteps = new ArrayList<>(16);
      overallListOfSteps.addAll(listOfStepsForward);
      overallListOfSteps.addAll(listOfStepsBackward);

      FootstepDataListMessage footstepsListMessage = HumanoidMessageTools.createFootstepDataListMessage(overallListOfSteps, 1.2, 0.8, ExecutionMode.QUEUE);
      footstepsListMessage.setDestination(PacketDestination.CONTROLLER);
      walkingStatusManager.sendFootstepList(footstepsListMessage);
   }

   private ArrayList<FootstepDataMessage> createFootstepList(RobotSide robotSide, boolean isDirectionForward)
   {
      ArrayList<FootstepDataMessage> listOfSteps = new ArrayList<>(8);

      for (int i = 0; i < listOfSteps.size(); i++)
      {
         double y = robotSide == RobotSide.LEFT ? 0.15 : -0.15;
         Point3D position = new Point3D();
         position.setY(y);

         if (isDirectionForward)
         {
            position.setX(i * 0.2 + 0.2);
         }
         else
         {
            double startingPoint = 8 * 0.2;
            position.setX(startingPoint -i * 0.2 - 0.2);
         }

         FootstepDataMessage footStep = HumanoidMessageTools.createFootstepDataMessage(robotSide, position, new Quaternion(0.0, 0.0, 0.0, 1.0));
         listOfSteps.set(i, footStep);
         robotSide = robotSide.getOppositeSide();
      }
      return listOfSteps;
   }
      
   private class WalkingStatusManager implements PacketConsumer<FootstepStatusMessage>
   {
      private final YoInteger footstepsCompleted = new YoInteger("footstepsCompleted", registry);
      private final YoInteger numberOfFootstepsToTake = new YoInteger("numberOfFootstepsToTake", registry);
      
      @Override
      public void receivedPacket(FootstepStatusMessage packet)
      {
         if(packet.getStatus() == FootstepStatus.COMPLETED.toByte())
            footstepsCompleted.increment();
      }
      
      public void sendFootstepList(FootstepDataListMessage footstepDataListMessage)
      {
         numberOfFootstepsToTake.set(footstepDataListMessage.getDataList().size());
         footstepsCompleted.set(0);
         packetCommunicator.send(footstepDataListMessage);
      }
      
      public boolean doneWalking()
      {
         return footstepsCompleted.getIntegerValue() == numberOfFootstepsToTake.getIntegerValue();
      }
   }
}
