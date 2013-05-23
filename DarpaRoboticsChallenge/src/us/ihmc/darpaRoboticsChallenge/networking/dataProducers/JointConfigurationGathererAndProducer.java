package us.ihmc.darpaRoboticsChallenge.networking.dataProducers;

import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.ReentrantLock;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.handControl.FingerJoint;
import us.ihmc.darpaRoboticsChallenge.handControl.SandiaHandModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.LongYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class JointConfigurationGathererAndProducer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LongYoVariable nextTimestamp = new LongYoVariable("nextTimestamp", registry);
   private final BooleanYoVariable hasAtlasAngles = new BooleanYoVariable("hasAtlasAngles", registry);
   private final BooleanYoVariable hasLeftHandAngles = new BooleanYoVariable("hasLeftHandAngles", registry);
   private final BooleanYoVariable hasRightHandAngles = new BooleanYoVariable("hasRightHandAngles", registry);
   private final SideDependentList<BooleanYoVariable> hasHandAngles = new SideDependentList<BooleanYoVariable>(hasLeftHandAngles, hasRightHandAngles);
   private final IntegerYoVariable delay = new IntegerYoVariable("jointConfigurationDelayInMs", registry);

   private final ObjectCommunicator objectCommunicator;

   private final ReentrantLock lock = new ReentrantLock(true);

   private int totalNumberOfJoints;
   private double[] allJointAngles;

   private final OneDoFJoint[] atlasJoints;
   private final SideDependentList<FingerJoint[]> handJoints = new SideDependentList<FingerJoint[]>();

   private final SixDoFJoint rootJoint;
   private final Vector3d rootTranslation = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();

   private final double[] atlasJointAngles;
   private final SideDependentList<double[]> handAngles = new SideDependentList<double[]>();

   private final ExecutorService sendPool = Executors.newSingleThreadExecutor(ThreadTools
         .getNamedThreadFactory("JointConfigurationGathererAndProducerSendPool"));
   private final JointConfigurationWorker jointConfigurationWorker = new JointConfigurationWorker();

   public JointConfigurationGathererAndProducer(ObjectCommunicator objectCommunicator, SDFFullRobotModel estimatorModel,
         SideDependentList<SandiaHandModel> handModels, YoVariableRegistry parentRegistry)
   {

      this.objectCommunicator = objectCommunicator;

      int numberOfAtlasJoints = DRCJointConfigurationData.atlasJointNames.length;
      this.atlasJoints = new OneDoFJoint[numberOfAtlasJoints];
      this.atlasJointAngles = new double[numberOfAtlasJoints];

      for (int i = 0; i < numberOfAtlasJoints; ++i)
      {
         atlasJoints[i] = estimatorModel.getOneDoFJointByName(DRCJointConfigurationData.atlasJointNames[i]);
      }

      this.rootJoint = estimatorModel.getRootJoint();

      totalNumberOfJoints += numberOfAtlasJoints;

      for (RobotSide robotSide : RobotSide.values)
      {
         HashMap<String, FingerJoint> jointsByName = new HashMap<String, FingerJoint>();
         for (FingerJoint joint : handModels.get(robotSide).getHandJoints())
         {
            jointsByName.put(joint.getName(), joint);
         }

         String[] handNames = robotSide == RobotSide.LEFT ? DRCJointConfigurationData.leftHandNames : DRCJointConfigurationData.rightHandNames;

         int numberOfHandJoints = handNames.length;
         handJoints.put(robotSide, new FingerJoint[numberOfHandJoints]);
         handAngles.put(robotSide, new double[numberOfHandJoints]);
         for (int i = 0; i < numberOfHandJoints; i++)
         {
            handJoints.get(robotSide)[i] = jointsByName.get(handNames[i]);
         }

         totalNumberOfJoints += numberOfHandJoints;
      }

      allJointAngles = new double[totalNumberOfJoints];

      nextTimestamp.set(Long.MIN_VALUE);
      hasAtlasAngles.set(false);
      hasLeftHandAngles.set(false);
      hasRightHandAngles.set(false);

      delay.set(DRCConfigParameters.JOINT_CONFIGURATION_RATE_IN_MS);

      parentRegistry.addChild(registry);
   }

   public void updateEstimatorJoints(long timestamp)
   {
      lock.lock();
      {
         if (!(timestamp >= nextTimestamp.getLongValue() && !hasAtlasAngles.getBooleanValue()))
         {
            lock.unlock();
            return;
         }
      }
      lock.unlock();

      for (int i = 0; i < atlasJoints.length; i++)
      {
         atlasJointAngles[i] = atlasJoints[i].getQ();
      }

      lock.lock();
      {
         hasAtlasAngles.set(true);
         rootJoint.packTranslation(rootTranslation);
         rootJoint.packRotation(rootOrientation);
         System.arraycopy(atlasJointAngles, 0, allJointAngles, 0, atlasJointAngles.length);
         sendIfNeeded(timestamp);
      }
      lock.unlock();

   }

   private void sendIfNeeded(long timestamp)
   {
      if (hasAtlasAngles.getBooleanValue() && hasLeftHandAngles.getBooleanValue() && hasRightHandAngles.getBooleanValue())
      {
         nextTimestamp.set(timestamp + TimeTools.milliSecondsToNanoSeconds(delay.getIntegerValue()));
         sendPool.execute(jointConfigurationWorker);
      }
   }

   public void updateHandJoints(RobotSide robotSide, long timestamp)
   {
      lock.lock();
      {
         if (!(timestamp >= nextTimestamp.getLongValue() && !hasHandAngles.get(robotSide).getBooleanValue()))
         {
            lock.unlock();
            return;
         }
      }
      lock.unlock();

      final FingerJoint[] sideHandJoints = handJoints.get(robotSide);
      final double[] sideHandAngles = handAngles.get(robotSide);
      for (int i = 0; i < sideHandJoints.length; i++)
      {
         sideHandAngles[i] = sideHandJoints[i].getQ();
      }

      int offset;
      if (robotSide == RobotSide.LEFT)
      {
         offset = atlasJointAngles.length;
      }
      else
      {
         offset = atlasJointAngles.length + handAngles.get(RobotSide.LEFT).length;
      }
      lock.lock();
      {
         hasHandAngles.get(robotSide).set(true);
         System.arraycopy(sideHandAngles, 0, allJointAngles, offset, sideHandAngles.length);
         sendIfNeeded(timestamp);
      }
      lock.unlock();

   }

   private class JointConfigurationWorker implements Runnable
   {
      private final DRCJointConfigurationData jointConfigurationData = new DRCJointConfigurationData();
      
      public void run()
      {
         try
         {
            lock.lock();
            jointConfigurationData.setJointAngles(allJointAngles);
            jointConfigurationData.setRootTranslation(rootTranslation);
            jointConfigurationData.setRootOrientation(rootOrientation);
            lock.unlock();
   
            objectCommunicator.consumeObject(jointConfigurationData);
              
            lock.lock();
            {
               jointConfigurationData.setJointAngles(allJointAngles);
               for (RobotSide robotSide : RobotSide.values)
               {
                  hasHandAngles.get(robotSide).set(false);
                  hasAtlasAngles.set(false);
               }
            }
            lock.unlock();
         }
         catch(Exception e)
         {
            e.printStackTrace();
         }
      }

   }
}
