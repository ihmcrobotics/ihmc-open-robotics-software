package us.ihmc.avatar.testTools;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This is using footstep poses for the ankle (only used in some old unit tests). Use footstep poses at the sole frame instead.
 */
@Deprecated
public class ScriptedFootstepGenerator
{
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();

   public ScriptedFootstepGenerator(HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      this.bipedFeet = setupBipedFeet(referenceFrames, fullRobotModel, walkingControllerParameters);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = bipedFeet.get(robotSide).getRigidBody();
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
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

   public SideDependentList<ContactablePlaneBody> setupBipedFeet(HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      double footForward, footBack, footWidth;

      footForward = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      footBack = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }
}
