package us.ihmc.avatar.testTools;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;

/**
 * This is using footstep poses for the ankle (only used in some old unit tests). Use footstep poses at the sole frame instead.
 */
@Deprecated
public class ScriptedFootstepGenerator
{
   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();

   public ScriptedFootstepGenerator(FullHumanoidRobotModel fullRobotModel)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         transformsFromAnkleToSole.put(robotSide, ankleToSole);
      }
   }

   public FootstepDataListMessage generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations)
   {
      return generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations, 0.0, 0.0);
   }

   public FootstepDataListMessage generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations, double swingTime, double transferTime)
   {
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);

      for (int i = 0; i < robotSides.length; i++)
      {
         RobotSide robotSide = robotSides[i];
         double[][] footstepLocationAndOrientation = footstepLocationsAndOrientations[i];
         Footstep footstep = generateFootstepFromLocationAndOrientation(robotSide, footstepLocationAndOrientation);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, footstep.getFootstepPose().getPosition(), footstep.getFootstepPose().getOrientation());
         footstepDataList.getFootstepDataList().add().set(footstepData);
      }

      return footstepDataList;
   }

   private Footstep generateFootstepFromLocationAndOrientation(RobotSide robotSide, double[][] footstepLocationAndOrientation)
   {
      double[] location = footstepLocationAndOrientation[0];
      double[] orientation = footstepLocationAndOrientation[1];

      return generateFootstepFromLocationAndOrientation(robotSide, location, orientation);
   }

   private Footstep generateFootstepFromLocationAndOrientation(RobotSide robotSide, double[] positionArray, double[] orientationArray)
   {
      Footstep footstep = new Footstep(robotSide);

      Point3D position = new Point3D(positionArray);
      Quaternion orientation = new Quaternion(orientationArray);
      RigidBodyTransform anklePose = new RigidBodyTransform();
      anklePose.getRotation().set(orientation);
      anklePose.getTranslation().set(position);
      FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), anklePose);

      footstep.setFromAnklePose(pose, transformsFromAnkleToSole.get(robotSide));

      return footstep;
   }

   public static RobotSide[] createRobotSidesStartingFrom(RobotSide robotSide, int length)
   {
      RobotSide[] ret = new RobotSide[length];
   
      for (int i = 0; i < length; i++)
      {
         ret[i] = robotSide;
         robotSide = robotSide.getOppositeSide();
      }
   
      return ret;
   }
}
