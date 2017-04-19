package us.ihmc.avatar.testTools;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * This is using footstep poses for the ankle (only used in some old unit tests). Use footstep poses at the sole frame instead.
 */
@Deprecated
public class ScriptedFootstepGenerator
{
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   private final SideDependentList<RigidBodyTransform> transformsFromSoleToAnkle = new SideDependentList<>();

   public ScriptedFootstepGenerator(HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      this.bipedFeet = setupBipedFeet(referenceFrames, fullRobotModel, walkingControllerParameters);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = bipedFeet.get(robotSide).getRigidBody();
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         RigidBodyTransform transformFromSoleToAnkle = new RigidBodyTransform();
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         soleFrame.getTransformToDesiredFrame(transformFromSoleToAnkle, ankleFrame);
         transformsFromSoleToAnkle.put(robotSide, transformFromSoleToAnkle);
      }
   }

   public FootstepDataListMessage generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations)
   {
      return generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations, 0.0, 0.0);
   }
   
   public FootstepDataListMessage generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations, double swingTime, double transferTime)
   {
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);

      for (int i = 0; i < robotSides.length; i++)
      {
         RobotSide robotSide = robotSides[i];
         double[][] footstepLocationAndOrientation = footstepLocationsAndOrientations[i];
         Footstep footstep = generateFootstepFromLocationAndOrientation(robotSide, footstepLocationAndOrientation);
         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, footstep.getFootstepPose().getPosition(),
                                                                    footstep.getFootstepPose().getOrientation());
         footstepData.setTrajectoryReferenceFrameId(ReferenceFrame.getWorldFrame());
         footstepDataList.add(footstepData);
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
      RigidBody foot = bipedFeet.get(robotSide).getRigidBody();
      boolean trustHeight = true;
      Point3D position = new Point3D(positionArray);
      Quaternion orientation = new Quaternion(orientationArray);
      
      RigidBodyTransform footstepPose = new RigidBodyTransform();
      footstepPose.setRotation(orientation);
      footstepPose.setTranslation(position);
      footstepPose.multiply(transformsFromSoleToAnkle.get(robotSide));
      
      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), footstepPose);
      Footstep footstep = new Footstep(foot, robotSide, pose, trustHeight);

      return footstep;
   }

   public SideDependentList<ContactablePlaneBody> setupBipedFeet(HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      double footForward, footBack, footWidth;

      footForward = walkingControllerParameters.getFootForwardOffset();
      footBack = walkingControllerParameters.getFootBackwardOffset();
      footWidth = walkingControllerParameters.getFootWidth();

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }
}
