package us.ihmc.valkyrie;

import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.GroundContactPoint;

public class ValkyrieInitialSetup implements DRCRobotInitialSetup<SDFRobot>
{
   private final double groundZ;
   private final double initialYaw;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();
   private final Quat4d rotation = new Quat4d();

   public ValkyrieInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   @Override
   public void initializeRobot(SDFRobot robot, DRCRobotJointMap jointMap)
   {
      robot.getOneDoFJoint("RightHipExtensor").setQ(-0.4);
      robot.getOneDoFJoint("RightKneeExtensor").setQ(-0.8);
      robot.getOneDoFJoint("RightAnkleExtensor").setQ(-0.4);
      
      robot.getOneDoFJoint("LeftHipExtensor").setQ(-0.4);
      robot.getOneDoFJoint("LeftKneeExtensor").setQ(-0.8);
      robot.getOneDoFJoint("LeftAnkleExtensor").setQ(-0.4);
      
      robot.getOneDoFJoint("LeftShoulderAdductor").setQ(-0.18);
      robot.getOneDoFJoint("LeftShoulderExtensor").setQ(0.3);
      robot.getOneDoFJoint("LeftElbowExtensor").setQ(-1.0);

      robot.getOneDoFJoint("RightShoulderAdductor").setQ(-0.18);
      robot.getOneDoFJoint("RightShoulderExtensor").setQ(0.3);
      robot.getOneDoFJoint("RightElbowExtensor").setQ(-1.0);
      
      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, offset);

      List<GroundContactPoint> contactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      double height = Double.POSITIVE_INFINITY;
      for(GroundContactPoint gc : contactPoints)
      {
         if(gc.getPositionPoint().getZ() < height)
         {
            height = gc.getPositionPoint().getZ();
         }
      }
      GroundContactPoint gc1 = contactPoints.get(0);
      double pelvisToFoot = offset.getZ() - height;

      // Hardcoded for gazebo integration
      //      double pelvisToFoot = 0.887;

      offset.setZ(groundZ + pelvisToFoot);

      //    offset.add(robot.getPositionInWorld());
      robot.setPositionInWorld(offset);

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
      double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
      yawPitchRoll[0] = initialYaw;
      frameOrientation.setYawPitchRoll(yawPitchRoll);

      robot.setOrientation(frameOrientation.getQuaternion());

      robot.update();
   }
}
