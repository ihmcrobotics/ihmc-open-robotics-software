package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Point3d;
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
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.util.NetworkPorts;
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
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final FullHumanoidRobotModel fullRobotModel;
   private final RigidBodyTransform pelvisToWorldTransform = new RigidBodyTransform();
   private final PlanarRegionsListListener planarRegionsListListener = new PlanarRegionsListListener();
   private final WalkingStatusManager walkingStatusManager = new WalkingStatusManager();

   private final Vector3f pelvisTranslation = new Vector3f();
   private final Quat4f pelvisOrientation = new Quat4f(0.0f, 0.0f, 0.0f, 1.0f);
   private final ReferenceFrame pelvisFrameFromRobotConfigurationDataPacket = new ReferenceFrame("pelvisFrame", ReferenceFrame.getWorldFrame())
   {
      private static final long serialVersionUID = 116774591450076114L;

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setTranslation(pelvisTranslation);
         transformToParent.setRotation(pelvisOrientation);
      }
   };
   
   private final DoubleYoVariable markersPositionX = new DoubleYoVariable("markersPositionX", registry);
   private final DoubleYoVariable markersPositionY = new DoubleYoVariable("markersPositionY", registry);
   private final DoubleYoVariable markersPositionZ = new DoubleYoVariable("markersPositionZ", registry);

   private final DoubleYoVariable markersYaw = new DoubleYoVariable("markersYaw", registry);
   private final DoubleYoVariable markersPitch = new DoubleYoVariable("markersPitch", registry);
   private final DoubleYoVariable markersRoll = new DoubleYoVariable("markersRoll", registry);

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
   private final DoubleYoVariable mocapWorldToRobotWorldTransformRoll = new DoubleYoVariable("mocapWorldToRobotWorldTransformZRoll", registry);

   private final BooleanYoVariable requestFootsteps = new BooleanYoVariable("requestFootsteps", registry);
   private final BooleanYoVariable walkingAround = new BooleanYoVariable("walkingAround", registry);
   
   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel)
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);

      packetCommunicator.attachListener(RobotConfigurationData.class, this);
      packetCommunicator.attachListener(PlanarRegionsListMessage.class, planarRegionsListListener);
      packetCommunicator.attachListener(FootstepStatus.class, walkingStatusManager);

      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("MocapModuleScheduler");
      LogModelProvider logModelProvider = drcRobotModel.getLogModelProvider();

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
         pelvisFrameFromRobotConfigurationDataPacket.update();
         MocapRigidBody pelvisRigidBody = getPelvisRigidBody(listOfRigidbodies);
         mocapToPelvisFrameConverter.initialize(pelvisFrameFromRobotConfigurationDataPacket, pelvisRigidBody);
      }
      else if (latestRobotConfigurationData == null)
      {
         PrintTools.info("Waiting for robot configuration data");
         return;
      }

      MocapRigidBody pelvisRigidBody = getPelvisRigidBody(listOfRigidbodies);
      sendPelvisTransformToController(pelvisRigidBody);
      setMarkersYoVariables(pelvisRigidBody);
            
      if (requestFootsteps.getBooleanValue())
      {
         planarRegionsListListener.onWalkingStarted();
         startWalking();
         walkingAround.set(true);
         requestFootsteps.set(false);
      }
      
      if(walkingAround.getBooleanValue() && walkingStatusManager.doneWalking())
      {
         planarRegionsListListener.onWalkingStopped();
         walkingAround.set(false);
      }
     
      yoVariableServer.update(System.currentTimeMillis());
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
      mocapToPelvisFrameConverter.computePelvisToWorldTransform(pelvisRigidBody, pelvisToWorldTransform);
      setPelvisYoVariables(pelvisToWorldTransform);

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

      pelvisTranslation.set(packet.getPelvisTranslation());
      pelvisOrientation.set(packet.getPelvisOrientation());

      rootJoint.setPosition(pelvisTranslation.getX(), pelvisTranslation.getY(), pelvisTranslation.getZ());
      rootJoint.setRotation(pelvisOrientation.getX(), pelvisOrientation.getY(), pelvisOrientation.getZ(), pelvisOrientation.getW());
      
      computeDriftTransform();

      rootJoint.getPredecessor().updateFramesRecursively();
      yoVariableServer.update(System.currentTimeMillis());
   }

   private void computeDriftTransform()
   {
      RigidBodyTransform driftTransform = mocapToPelvisFrameConverter.getMocapFrame().getTransformToDesiredFrame(pelvisFrameFromRobotConfigurationDataPacket);
      
      Vector3d driftTranslation = new Vector3d();
      driftTransform.getTranslation(driftTranslation);
      
      Quat4d driftRotation = new Quat4d();
      driftTransform.getRotation(driftRotation);
      double[] driftRotationYPR = new double[3];
      RotationTools.convertQuaternionToYawPitchRoll(driftRotation, driftRotationYPR);
      
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
         Point3d position = new Point3d();
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

         FootstepDataMessage footStep = new FootstepDataMessage(robotSide, position, new Quat4d(0.0, 0.0, 0.0, 1.0));
         listOfSteps.set(i, footStep);
         robotSide = robotSide.getOppositeSide();
      }
      return listOfSteps;
   }
   
   private class PlanarRegionsListListener implements PacketConsumer<PlanarRegionsListMessage>
   {
      private PlanarRegionsListMessage latestPlanarRegionsListMessage;
      private PlanarRegionsListMessage beforeWalkingPlanarRegionsListMessage;
      private PlanarRegionsListMessage afterWalkingPlanarRegionsListMessage;
      
      @Override
      public void receivedPacket(PlanarRegionsListMessage packet)
      {         
         this.latestPlanarRegionsListMessage = packet;
      }
      
      public void onWalkingStarted()
      {
         beforeWalkingPlanarRegionsListMessage = latestPlanarRegionsListMessage;
      }

      public void onWalkingStopped()
      {
         afterWalkingPlanarRegionsListMessage = latestPlanarRegionsListMessage;
      }
      
      public PlanarRegionsListMessage getPlanarRegionsBeforeWalking()
      {
         return beforeWalkingPlanarRegionsListMessage;
      }

      public PlanarRegionsListMessage getPlanarRegionsAfterWalking()
      {
         return afterWalkingPlanarRegionsListMessage;
      }
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
