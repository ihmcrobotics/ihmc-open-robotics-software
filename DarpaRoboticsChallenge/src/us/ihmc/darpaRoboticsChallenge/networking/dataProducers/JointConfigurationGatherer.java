package us.ihmc.darpaRoboticsChallenge.networking.dataProducers;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.producers.DRCJointConfigurationData;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

// packs joint positions
public class JointConfigurationGatherer
{  
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OneDoFJoint[] joints;

   private final SixDoFJoint rootJoint;
   private final Vector3d rootTranslation = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();

   public JointConfigurationGatherer(SDFFullRobotModel estimatorModel, YoVariableRegistry parentRegistry, DRCRobotJointMap jointMap)
   {
      parentRegistry.addChild(registry);
      this.rootJoint = estimatorModel.getRootJoint();
      String[] jointNames = jointMap.getOrderedJointNames();
      int numberOfJoints = jointNames.length;
      this.joints = new OneDoFJoint[numberOfJoints];

      for (int i = 0; i < numberOfJoints; ++i)
      {
         joints[i] = estimatorModel.getOneDoFJointByName(jointNames[i]);
      }
   }

   // fills a DRCJointConfigurationData object on the ConcurrentRingBuffer
   public void packEstimatorJoints(long timestamp, DRCJointConfigurationData jointConfigurationData, int numberOfJoints)
   {
      if (jointConfigurationData == null)
      {
         return;
      }
      
      if(jointConfigurationData.jointAngles == null)
      {
         jointConfigurationData.initJointNumber(numberOfJoints);
      }

      double[] jointAngles = jointConfigurationData.getJointAngles();

      rootJoint.packTranslation(rootTranslation);
      rootJoint.packRotation(rootOrientation);

      for (int i = 0; i < joints.length; i++)
      {
         jointAngles[i] = joints[i].getQ();
      }

      jointConfigurationData.setRootTranslation(rootTranslation);
      jointConfigurationData.setRootOrientation(rootOrientation);
      jointConfigurationData.setSimTime(timestamp);
   }
}
