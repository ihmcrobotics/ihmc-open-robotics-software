package us.ihmc.humanoidRobotics.frames;

import java.util.EnumMap;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.RobotSpecificJointNames;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.containers.ContainerTools;

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
   private final FullHumanoidRobotModel fullRobotModel;

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
   private final ReferenceFrame midFeetZUpWalkDirectionFrame;
   private final ReferenceFrame midFeetUnderPelvisWalkDirectionFrame;

   private final ReferenceFrame centerOfMassFrame;

   public HumanoidReferenceFrames(FullHumanoidRobotModel fullRobotModel) {
      this.fullRobotModel = fullRobotModel;

      pelvisFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
//      chestFrame = fullRobotModel.getChest().getParentJoint().getFrameAfterJoint();
      chestFrame = null;
      pelvisZUpFrame = new ZUpFrame(worldFrame, pelvisFrame, "pelvisZUpFrame");

      RobotSpecificJointNames robotJointNames = fullRobotModel.getRobotSpecificJointNames();

      if (robotJointNames.getNeckJointNames() != null) {
         for (NeckJointName neckJointName : robotJointNames.getNeckJointNames()) {
            this.neckReferenceFrames.put(neckJointName, fullRobotModel.getNeckJoint(neckJointName).getFrameAfterJoint());
         }
      }

      if (robotJointNames.getSpineJointNames() != null) {
         for (SpineJointName spineJointName : robotJointNames.getSpineJointNames()) {
            this.spineReferenceFrames.put(spineJointName, fullRobotModel.getSpineJoint(spineJointName).getFrameAfterJoint());
         }
      }

      if (robotJointNames.getArmJointNames() != null) {
         for (RobotSide robotSide : RobotSide.values) {
            for (ArmJointName armJointName : robotJointNames.getArmJointNames()) {
               this.armJointFrames.get(robotSide).put(armJointName, fullRobotModel.getArmJoint(robotSide, armJointName).getFrameAfterJoint());
            }
         }
      }

      if (robotJointNames.getLegJointNames() != null)
      {
         for (RobotSide robotSide : RobotSide.values) {
            for (LegJointName legJointName : robotJointNames.getLegJointNames()) {
               ReferenceFrame legJointFrame = fullRobotModel.getFrameAfterLegJoint(robotSide, legJointName);
               legJointFrames.get(robotSide).put(legJointName, legJointFrame);
            }
         }
      }

      for (RobotSide robotSide : RobotSide.values)
     {
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, getFootFrame(robotSide), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         soleFrames.put(robotSide, fullRobotModel.getSoleFrame(robotSide));
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUp", pelvisZUpFrame, getFootFrame(RobotSide.LEFT), getFootFrame(RobotSide.RIGHT));

      //this is a frame that is directly between the 2 feet but faces forward instead of perpendicular to the line between the feet
      midFeetZUpWalkDirectionFrame = new ReferenceFrame("midFeetZUpWalkDirectionFrame", ReferenceFrame.getWorldFrame())
      {
         private static final long serialVersionUID = 8156971218482020008L;
         private final FramePose midFootZUpPose = new FramePose();
         private final FramePose leftFootPose = new FramePose();
         private final FramePose rightFootPose = new FramePose();
         private final SideDependentList<FramePose> footPoses = new SideDependentList<>(leftFootPose, rightFootPose);

         private final RigidBodyTransform tempTransform = new RigidBodyTransform();
         private final double[] tempYawPitchRoll = new double[3];

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               ReferenceFrame footFrame = getFootFrame(robotSide);
               footFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
               FramePose footPose = footPoses.get(robotSide);
               footPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), tempTransform);
               footPose.getOrientation(tempYawPitchRoll);
               tempYawPitchRoll[1] = 0.0;
               tempYawPitchRoll[2] = 0.0;
               footPose.setOrientation(tempYawPitchRoll);
            }

            midFootZUpPose.interpolate(leftFootPose, rightFootPose, 0.5);
            midFootZUpPose.setZ(Math.min(leftFootPose.getZ(), rightFootPose.getZ()));
            midFootZUpPose.getPose(transformToParent);
         }
      };
      // this is a
      midFeetUnderPelvisWalkDirectionFrame = new ReferenceFrame("midFeetUnderPelvisWalkDirectionFrame", ReferenceFrame.getWorldFrame())
      {
         private static final long serialVersionUID = -8209218692291863332L;
         private final FramePose midFeetPose = new FramePose();
         private final FramePose pelvisPose = new FramePose();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            midFeetPose.setToZero(midFeetZUpWalkDirectionFrame);
            pelvisPose.setToZero(getPelvisFrame());

            pelvisPose.changeFrame(midFeetZUpWalkDirectionFrame);
            midFeetPose.setX(pelvisPose.getX());
            midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());
            midFeetPose.getPose(transformToParent);
         }
      };

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
   /**
    * Return the ReferenceFrame located between the feet (to remove robot swaying left to right) 
    * but under the pelvis (to remove forward and backwards swaying)
    */
   @Override
   public ReferenceFrame getMidFeetUnderPelvisFrame()
   {
      return midFeetUnderPelvisWalkDirectionFrame;
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
      midFeetZUpWalkDirectionFrame.update();
      midFeetUnderPelvisWalkDirectionFrame.update();

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
