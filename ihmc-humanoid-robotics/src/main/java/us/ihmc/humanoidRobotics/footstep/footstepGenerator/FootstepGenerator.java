package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public interface FootstepGenerator
{
   ArrayList<Footstep> generateDesiredFootstepList();

   default Footstep generateFootstepWithoutHeightMap(FramePose2DReadOnly footPose2d, RobotSide robotSide, double height,
                                                    Vector3D planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3D position = new Point3D(footPose2d.getX(), footPose2d.getY(), height);
      Quaternion orientation = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);

      FramePose3D solePose = new FramePose3D(ReferenceFrame.getWorldFrame(), position, orientation);
      Footstep footstep = new Footstep(robotSide, solePose);

      return footstep;
   }
}
