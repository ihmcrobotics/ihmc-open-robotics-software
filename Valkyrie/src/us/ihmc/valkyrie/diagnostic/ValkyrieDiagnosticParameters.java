package us.ihmc.valkyrie.diagnostic;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieDiagnosticParameters extends DiagnosticParameters
{
   private final DRCRobotJointMap jointMap;
   private final ValkyrieSensorInformation sensorInformation;
   
   private final boolean ignoreAllNeckJoints = true;
   private final boolean ignoreAllArmJoints = false;
   private final boolean ignoreAllLegJoints = false;
   private final boolean ignoreAllSpineJoints = true;

   public ValkyrieDiagnosticParameters(DiagnosticEnvironment diagnosticEnvironment, ValkyrieRobotModel robotModel)
   {
      super(diagnosticEnvironment);
      this.jointMap = robotModel.getJointMap();
      this.sensorInformation = robotModel.getSensorInformation();
   }

   @Override
   public List<String> getJointsToIgnoreDuringDiagnostic()
   {
      List<String> jointToIgnoreList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = ValkyrieOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbRoll]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch3]);
      }

      if (ignoreAllNeckJoints)
      {
         for (NeckJointName neckJointName : jointMap.getNeckJointNames())
            jointToIgnoreList.add(jointMap.getNeckJointName(neckJointName));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (ignoreAllArmJoints)
         {
            for (ArmJointName armJointName : jointMap.getArmJointNames())
               jointToIgnoreList.add(jointMap.getArmJointName(robotSide, armJointName));
         }
         else
         {
            jointToIgnoreList.add(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_YAW));
            jointToIgnoreList.add(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL));
            jointToIgnoreList.add(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH));
         }

         if (ignoreAllLegJoints)
         {
            for (LegJointName legJointName : jointMap.getLegJointNames())
               jointToIgnoreList.add(jointMap.getLegJointName(robotSide, legJointName));
         }
         else
         {
//            jointToIgnoreList.add(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW));
//            jointToIgnoreList.add(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL));
//            jointToIgnoreList.add(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH));
//            jointToIgnoreList.add(jointMap.getLegJointName(robotSide, LegJointName.KNEE));
//            jointToIgnoreList.add(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH));
         }
      }

      if (ignoreAllSpineJoints)
      {
         for (SpineJointName spineJointName : jointMap.getSpineJointNames())
            jointToIgnoreList.add(jointMap.getSpineJointName(spineJointName));
      }
      else
      {
         
      }

      jointToIgnoreList.add("hokuyo_joint");
      
      return jointToIgnoreList;
   }

   @Override
   public String getPelvisIMUName()
   {
      return sensorInformation.getPrimaryBodyImu();
   }
}
