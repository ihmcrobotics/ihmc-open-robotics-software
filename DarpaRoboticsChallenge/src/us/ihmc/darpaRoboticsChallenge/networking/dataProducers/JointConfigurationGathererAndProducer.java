package us.ihmc.darpaRoboticsChallenge.networking.dataProducers;

import java.util.HashMap;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.darpaRoboticsChallenge.handControl.FingerJoint;
import us.ihmc.darpaRoboticsChallenge.handControl.SandiaHandModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.remote.serialization.JointConfigurationData;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class JointConfigurationGathererAndProducer
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ObjectCommunicator objectCommunicator;

   private final OneDoFJoint[] atlasJoints;
   private final SideDependentList<FingerJoint[]> handJoints = new SideDependentList<FingerJoint[]>();

   private final SixDoFJoint rootJoint;
   private final Vector3d rootTranslation = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();

   private final SideDependentList<ConcurrentCopier<double[]>> handAngles = new SideDependentList<ConcurrentCopier<double[]>>();
   private final ConcurrentRingBuffer<DRCJointConfigurationData> stateRingBuffer;

   public JointConfigurationGathererAndProducer(ObjectCommunicator objectCommunicator, SDFFullRobotModel estimatorModel,
         SideDependentList<SandiaHandModel> handModels, YoVariableRegistry parentRegistry)
   {

      parentRegistry.addChild(registry);
      this.objectCommunicator = objectCommunicator;
      this.rootJoint = estimatorModel.getRootJoint();

      // Setup Atlas Joints
      int numberOfAtlasJoints = DRCJointConfigurationData.atlasJointNames.length;
      this.atlasJoints = new OneDoFJoint[numberOfAtlasJoints];

      for (int i = 0; i < numberOfAtlasJoints; ++i)
      {
         atlasJoints[i] = estimatorModel.getOneDoFJointByName(DRCJointConfigurationData.atlasJointNames[i]);
      }

      // Setup hand joints
      for (RobotSide robotSide : RobotSide.values)
      {
         HashMap<String, FingerJoint> jointsByName = new HashMap<String, FingerJoint>();
         for (FingerJoint joint : handModels.get(robotSide).getHandJoints())
         {
            jointsByName.put(joint.getName(), joint);
         }

         String[] handNames = robotSide == RobotSide.LEFT ? DRCJointConfigurationData.leftHandNames : DRCJointConfigurationData.rightHandNames;

         final int numberOfHandJoints = handNames.length;
         handJoints.put(robotSide, new FingerJoint[numberOfHandJoints]);
         for (int i = 0; i < numberOfHandJoints; i++)
         {
            handJoints.get(robotSide)[i] = jointsByName.get(handNames[i]);
         }

         final Builder<double[]> handAngleBuilder = new Builder<double[]>()
         {
            public double[] newInstance()
            {
               return new double[numberOfHandJoints];
            }
         };

         handAngles.set(robotSide, new ConcurrentCopier<double[]>(handAngleBuilder));
      }

      // Setup ring buffer
      Builder<DRCJointConfigurationData> jointConfigurationBuilder = new Builder<DRCJointConfigurationData>()
      {
         public DRCJointConfigurationData newInstance()
         {
            return new DRCJointConfigurationData();
         }
      };
      stateRingBuffer = new ConcurrentRingBuffer<DRCJointConfigurationData>(jointConfigurationBuilder, 8);

      // Start thread
      new Thread(new JointConfigurationWorker()).start();
   }

   
   // upon receiving new atlas joint data, pulls the latest hand states and fills a
   // DRCJointConfigurationData object on the ConcurrentRingBuffer
   public void updateEstimatorJoints(long timestamp)
   {
      rootJoint.packTranslation(rootTranslation);
      rootJoint.packRotation(rootOrientation);

      DRCJointConfigurationData jointConfigurationData = stateRingBuffer.next();
      if (jointConfigurationData == null)
      {
         return;
      }

      double[] jointAngles = jointConfigurationData.getJointAngles();
      for (int i = 0; i < atlasJoints.length; i++)
      {
         jointAngles[i] = atlasJoints[i].getQ();
      }

      double[] leftHandAngles = handAngles.get(RobotSide.LEFT).getCopyForReading();
      double[] rightHandAngles = handAngles.get(RobotSide.LEFT).getCopyForReading();
      if (leftHandAngles != null && rightHandAngles != null)
      {
         System.arraycopy(leftHandAngles, 0, jointAngles, atlasJoints.length, leftHandAngles.length);
         System.arraycopy(rightHandAngles, 0, jointAngles, atlasJoints.length + leftHandAngles.length, rightHandAngles.length);
      }

      jointConfigurationData.setRootTranslation(rootTranslation);
      jointConfigurationData.setRootOrientation(rootOrientation);
      jointConfigurationData.setSimTime(timestamp);

      stateRingBuffer.commit();
   }

   // uses ConcurrentCopier to keep the latest hand joint state for each hand
   public void updateHandJoints(RobotSide robotSide, long timestamp)
   {
      final FingerJoint[] sideHandJoints = handJoints.get(robotSide);
      final double[] sideHandAngles = handAngles.get(robotSide).getCopyForWriting();
      for (int i = 0; i < sideHandJoints.length; i++)
      {
         sideHandAngles[i] = sideHandJoints[i].getQ();
      }
      handAngles.get(robotSide).commit();
   }

   // feeds the RingBuffer into the objectCommunicator
   private class JointConfigurationWorker implements Runnable
   {
      public void run()
      {
         while (true)
         {
            if (stateRingBuffer.poll())
            {
               DRCJointConfigurationData state;
               while ((state = stateRingBuffer.read()) != null)
               {
                  objectCommunicator.consumeObject(state);
               }
               stateRingBuffer.flush();
            }

            ThreadTools.sleep(WORKER_SLEEP_TIME_MILLIS);
         }
      }
   }
}
