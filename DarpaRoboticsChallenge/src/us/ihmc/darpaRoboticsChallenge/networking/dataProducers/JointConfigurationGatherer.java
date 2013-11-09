package us.ihmc.darpaRoboticsChallenge.networking.dataProducers;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.darpaRoboticsChallenge.handControl.FingerJoint;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

// collects Atlas joint positions and Hand positions and packs them together
public class JointConfigurationGatherer
{  
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OneDoFJoint[] atlasJoints;
   private final SideDependentList<FingerJoint[]> handJoints = new SideDependentList<FingerJoint[]>();

   private final SixDoFJoint rootJoint;
   private final Vector3d rootTranslation = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();

   private final SideDependentList<ConcurrentCopier<double[]>> handAngles = new SideDependentList<ConcurrentCopier<double[]>>();

   public JointConfigurationGatherer(SDFFullRobotModel estimatorModel, SideDependentList<ArrayList<FingerJoint>> handModels, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
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
         final int numberOfHandJoints;
         if (handModels != null)
         {
            HashMap<String, FingerJoint> jointsByName = new HashMap<String, FingerJoint>();
            for (FingerJoint joint : handModels.get(robotSide))
            {
               jointsByName.put(joint.getName().toLowerCase(), joint);
            }

            String[] handNames = robotSide == RobotSide.LEFT ? DRCJointConfigurationData.leftHandNames : DRCJointConfigurationData.rightHandNames;

            numberOfHandJoints = handNames.length;
            handJoints.put(robotSide, new FingerJoint[numberOfHandJoints]);
            for (int i = 0; i < numberOfHandJoints; i++)
            {
               handJoints.get(robotSide)[i] = jointsByName.get(handNames[i].toLowerCase());
            }
         }
         else
         {
            numberOfHandJoints = 0;
         }

         final Builder<double[]> handAngleBuilder = new Builder<double[]>()
         {
            @Override
            public double[] newInstance()
            {
               return new double[numberOfHandJoints];
            }
         };

         handAngles.set(robotSide, new ConcurrentCopier<double[]>(handAngleBuilder));

      }
   }

   // upon receiving new atlas joint data, pulls the latest hand states and fills a
   // DRCJointConfigurationData object on the ConcurrentRingBuffer
   public void packEstimatorJoints(long timestamp, DRCJointConfigurationData jointConfigurationData)
   {
      if (jointConfigurationData == null)
      {
         return;
      }

      double[] jointAngles = jointConfigurationData.getJointAngles();

      double[] leftHandAngles = handAngles.get(RobotSide.LEFT).getCopyForReading();
      double[] rightHandAngles = handAngles.get(RobotSide.RIGHT).getCopyForReading();

      rootJoint.packTranslation(rootTranslation);
      rootJoint.packRotation(rootOrientation);

      for (int i = 0; i < atlasJoints.length; i++)
      {
         jointAngles[i] = atlasJoints[i].getQ();
      }

      if ((leftHandAngles != null) && (rightHandAngles != null))
      {
         System.arraycopy(leftHandAngles, 0, jointAngles, atlasJoints.length, leftHandAngles.length);
         System.arraycopy(rightHandAngles, 0, jointAngles, atlasJoints.length + leftHandAngles.length, rightHandAngles.length);
      }

      jointConfigurationData.setRootTranslation(rootTranslation);
      jointConfigurationData.setRootOrientation(rootOrientation);
      jointConfigurationData.setSimTime(timestamp);
   }

   // uses ConcurrentCopier to keep the latest hand joint state for each hand
   public void updateHandJoints(RobotSide robotSide, long timestamp)
   {
      if (!handJoints.isEmpty())
      {
         final FingerJoint[] sideHandJoints = handJoints.get(robotSide);
         final double[] sideHandAngles = handAngles.get(robotSide).getCopyForWriting();
         for (int i = 0; i < sideHandJoints.length; i++)
         {
        	 if(sideHandJoints[i] != null)
        	 {
        		 sideHandAngles[i] = sideHandJoints[i].getQ();
        	 }
         }
         handAngles.get(robotSide).commit();
      }
   }
}
