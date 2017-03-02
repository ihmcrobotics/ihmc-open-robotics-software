package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.io.IOException;
import java.util.ArrayList;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
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
      private static final long serialVersionUID = 116774591450076114L;

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
      private static final long serialVersionUID = 116774591450076114L;

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(pelvisToWorldFromMocapData);
      }
   };
   
   private final DoubleYoVariable computedPelvisPositionX = new DoubleYoVariable("computedPelvisPositionX", registry);
   private final DoubleYoVariable computedPelvisPositionY = new DoubleYoVariable("computedPelvisPositionY", registry);
   private final DoubleYoVariable computedPelvisPositionZ = new DoubleYoVariable("computedPelvisPositionZ", registry);

   private final DoubleYoVariable computedPelvisYaw = new DoubleYoVariable("computedPelvisYaw", registry);
   private final DoubleYoVariable computedPelvisPitch = new DoubleYoVariable("computedPelvisPitch", registry);
   private final DoubleYoVariable computedPelvisRoll = new DoubleYoVariable("computedPelvisRoll", registry);
   
   private final DoubleYoVariable mocapWorldToRobotWorldTransformX = new DoubleYoVariable("mocapWorldToRobotWorldTransformX", registry);
   private final DoubleYoVariable mocapWorldToRobotWorldTransformY = new DoubleYoVariable("mocapWorldToRobotWorldTransformY", registry);
   private final DoubleYoVariable mocapWorldToRobotWorldTransformZ = new DoubleYoVariable("mocapWorldToRobotWorldTransformZ", registry);
   private final DoubleYoVariable mocapWorldToRobotWorldTransformYaw = new DoubleYoVariable("mocapWorldToRobotWorldTransformYaw", registry);
   private final DoubleYoVariable mocapWorldToRobotWorldTransformPitch = new DoubleYoVariable("mocapWorldToRobotWorldTransformPitch", registry);
   private final DoubleYoVariable mocapWorldToRobotWorldTransformRoll = new DoubleYoVariable("mocapWorldToRobotWorldTransformRoll", registry);

   private final BooleanYoVariable requestReInitialization = new BooleanYoVariable("requestReInitialization", registry);
   private final BooleanYoVariable requestFootsteps = new BooleanYoVariable("requestFootsteps", registry);
   private final BooleanYoVariable walkingAround = new BooleanYoVariable("walkingAround", registry);
      
   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel, MocapPlanarRegionsListManager planarRegionsListManager)
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);

      packetCommunicator.attachListener(RobotConfigurationData.class, this);
      packetCommunicator.attachListener(FootstepStatus.class, walkingStatusManager);

      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("MocapModuleScheduler");
      LogModelProvider logModelProvider = drcRobotModel.getLogModelProvider();
      
      this.planarRegionsListManager = planarRegionsListManager;

      yoVariableServer = new YoVariableServer(getClass(), scheduler, logModelProvider, drcRobotModel.getLogSettings(), MOCAP_SERVER_DT);
      fullRobotModel = drcRobotModel.createFullRobotModel();
      yoVariableServer.setMainRegistry(registry, fullRobotModel, null);

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

      FootstepDataListMessage footstepsListMessage = new FootstepDataListMessage(overallListOfSteps, 1.2, 0.8, ExecutionMode.QUEUE);
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

         FootstepDataMessage footStep = new FootstepDataMessage(robotSide, position, new Quaternion(0.0, 0.0, 0.0, 1.0));
         listOfSteps.set(i, footStep);
         robotSide = robotSide.getOppositeSide();
      }
      return listOfSteps;
   }
      
   private class WalkingStatusManager implements PacketConsumer<FootstepStatus>
   {
      private final IntegerYoVariable footstepsCompleted = new IntegerYoVariable("footstepsCompleted", registry);
      private final IntegerYoVariable numberOfFootstepsToTake = new IntegerYoVariable("numberOfFootstepsToTake", registry);
      
      @Override
      public void receivedPacket(FootstepStatus packet)
      {
         if(packet.getStatus().equals(FootstepStatus.Status.COMPLETED))
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
