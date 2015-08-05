package us.ihmc.humanoidRobotics.frames;

import java.util.EnumMap;

import us.ihmc.robotics.containers.ContainerTools;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.LimbName;
import us.ihmc.humanoidRobotics.partNames.NeckJointName;
import us.ihmc.humanoidRobotics.partNames.RobotSpecificJointNames;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class HumanoidReferenceFrames implements CommonHumanoidReferenceFrames
{
   private final FullRobotModel fullRobotModel;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame chestFrame;
   private final ReferenceFrame pelvisFrame;
   private final ZUpFrame pelvisZUpFrame;

   private final EnumMap<SpineJointName, ReferenceFrame> spineReferenceFrames = ContainerTools.createEnumMap(SpineJointName.class);
   private final EnumMap<NeckJointName, ReferenceFrame> neckReferenceFrames = ContainerTools.createEnumMap(NeckJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, ReferenceFrame>> armJointFrames = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final SideDependentList<EnumMap<LegJointName, ReferenceFrame>> legJointFrames = SideDependentList.createListOfEnumMaps(LegJointName.class);

   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();
   private final MidFrameZUpFrame midFeetZUpFrame;
   private final ReferenceFrame centerOfMassFrame;

   public HumanoidReferenceFrames(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;

      pelvisFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      chestFrame = fullRobotModel.getChest().getParentJoint().getFrameAfterJoint();
      pelvisZUpFrame = new ZUpFrame(worldFrame, pelvisFrame, "pelvisZUpFrame");

      RobotSpecificJointNames robotJointNames = fullRobotModel.getRobotSpecificJointNames();

      for (NeckJointName neckJointName : robotJointNames.getNeckJointNames())
      {
         this.neckReferenceFrames.put(neckJointName, fullRobotModel.getNeckJoint(neckJointName).getFrameAfterJoint());
      }

      for (SpineJointName spineJointName : robotJointNames.getSpineJointNames())
      {
         this.spineReferenceFrames.put(spineJointName, fullRobotModel.getSpineJoint(spineJointName).getFrameAfterJoint());
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : robotJointNames.getArmJointNames())
         {
            this.armJointFrames.get(robotSide).put(armJointName, fullRobotModel.getArmJoint(robotSide, armJointName).getFrameAfterJoint());
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : robotJointNames.getLegJointNames())
         {
            ReferenceFrame legJointFrame = fullRobotModel.getFrameAfterLegJoint(robotSide, legJointName);
            legJointFrames.get(robotSide).put(legJointName, legJointFrame);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, getFootFrame(robotSide), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         soleFrames.put(robotSide, fullRobotModel.getSoleFrame(robotSide));
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUp", pelvisZUpFrame, getFootFrame(RobotSide.LEFT), getFootFrame(RobotSide.RIGHT));

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, fullRobotModel.getElevator());
   }

   @Override
   public SideDependentList<ReferenceFrame> getAnkleZUpReferenceFrames()
   {
      return ankleZUpFrames;
   }

   @Override
   public SideDependentList<ReferenceFrame> getFootReferenceFrames()
   {
      return new SideDependentList<ReferenceFrame>(getFootFrame(RobotSide.LEFT), getFootFrame(RobotSide.RIGHT));
   }

   @Override
   public ReferenceFrame getFootFrame(RobotSide robotSide)
   {
      return fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
   }

   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   // TORSO
   public ReferenceFrame getSpineFrame(SpineJointName spineJointName)
   {
      return spineReferenceFrames.get(spineJointName);
   }

   @Override
   public ReferenceFrame getABodyAttachedZUpFrame()
   {
      return pelvisZUpFrame;
   }

   /**
    * Return the ReferenceFrame located after the parent joint of the pelvis.
    */
   @Override
   public ReferenceFrame getPelvisZUpFrame()
   {
      return pelvisZUpFrame;
   }

   /**
    * Return the ReferenceFrame located after the parent joint of the chest.
    */
   public ReferenceFrame getChestFrame()
   {
      return chestFrame;
   }

   @Override
   public ReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }

   public ReferenceFrame getNeckFrame(NeckJointName neckJointName)
   {
      return neckReferenceFrames.get(neckJointName);
   }

   public ReferenceFrame getArmFrame(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointFrames.get(robotSide).get(armJointName);
   }

   // LEGS
   @Override
   public ReferenceFrame getLegJointFrame(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointFrames.get(robotSide).get(legJointName);
   }

   public ReferenceFrame getHipYawFrame(RobotSide robotSide)
   {
      return getLegJointFrame(robotSide, LegJointName.HIP_YAW);
   }

   public ReferenceFrame getHipRollFrame(RobotSide robotSide)
   {
      return getLegJointFrame(robotSide, LegJointName.HIP_ROLL);
   }

   public ReferenceFrame getHipPitchFrame(RobotSide robotSide)
   {
      return getLegJointFrame(robotSide, LegJointName.HIP_PITCH);
   }

   public ReferenceFrame getKneeFrame(RobotSide robotSide)
   {
      return getLegJointFrame(robotSide, LegJointName.KNEE);
   }

   public ReferenceFrame getAnkleRollFrame(RobotSide robotSide)
   {
      return getLegJointFrame(robotSide, LegJointName.ANKLE_ROLL);
   }

   public ReferenceFrame getAnklePitchFrame(RobotSide robotSide)
   {
      return getLegJointFrame(robotSide, LegJointName.ANKLE_PITCH);
   }

   @Override
   public ReferenceFrame getAnkleZUpFrame(RobotSide robotSide)
   {
      return ankleZUpFrames.get(robotSide);
   }

   @Override
   public ReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUpFrame;
   }

   public ReferenceFrame getHandFrame(RobotSide robotSide)
   {
      return fullRobotModel.getHandControlFrame(robotSide);
   }
   
   @Override
   public void updateFrames()
   {
      fullRobotModel.updateFrames();

      pelvisZUpFrame.update();

      midFeetZUpFrame.update();

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleZUpFrames.get(robotSide).update();
         soleFrames.get(robotSide).update();
      }

      centerOfMassFrame.update();
   }

   @Override
   public EnumMap<LegJointName, ReferenceFrame> getLegJointFrames(RobotSide robotSide)
   {
      return legJointFrames.get(robotSide);
   }

   @Override
   public ReferenceFrame getIMUFrame()
   {
      return pelvisFrame;
   }

   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   @Override
   public ReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   @Override
   public SideDependentList<ReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }
}
