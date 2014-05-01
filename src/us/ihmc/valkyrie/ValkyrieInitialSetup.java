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
   private double groundZ;
   private double initialYaw;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d positionInWorld = new Vector3d();
   private final Vector3d offset = new Vector3d();
   private final Quat4d rotation = new Quat4d();
   private boolean robotInitialized = false;

   public ValkyrieInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   @Override
   public void initializeRobot(SDFRobot robot, DRCRobotJointMap jointMap)
   {
      if(!robotInitialized)
      {
         setActuatorPositions(robot);
         positionRobotInWorld(robot);
         robotInitialized = true;
      }
   }
   
   private void setActuatorPositions(SDFRobot robot)
   {
      robot.getOneDegreeOfFreedomJoint("RightHipExtensor").setQ(-0.4);
      robot.getOneDegreeOfFreedomJoint("RightKneeExtensor").setQ(-0.8);
      robot.getOneDegreeOfFreedomJoint("RightAnkleExtensor").setQ(-0.4);
      
      robot.getOneDegreeOfFreedomJoint("LeftHipExtensor").setQ(-0.4);
      robot.getOneDegreeOfFreedomJoint("LeftKneeExtensor").setQ(-0.8);
      robot.getOneDegreeOfFreedomJoint("LeftAnkleExtensor").setQ(-0.4);
      
      robot.getOneDegreeOfFreedomJoint("LeftShoulderAdductor").setQ(-0.18);
      robot.getOneDegreeOfFreedomJoint("LeftShoulderExtensor").setQ(0.3);
      robot.getOneDegreeOfFreedomJoint("LeftElbowExtensor").setQ(-1.0);
      
      robot.getOneDegreeOfFreedomJoint("RightShoulderAdductor").setQ(-0.18);
      robot.getOneDegreeOfFreedomJoint("RightShoulderExtensor").setQ(0.3);
      robot.getOneDegreeOfFreedomJoint("RightElbowExtensor").setQ(-1.0);
      robot.update();
   }
   
   private void positionRobotInWorld(SDFRobot robot)
   {
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, positionInWorld);
      positionInWorld.setZ(groundZ + getPelvisToFoot(robot));
      positionInWorld.add(offset);
      robot.setPositionInWorld(positionInWorld);
      
      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
      double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
      yawPitchRoll[0] = initialYaw;
      frameOrientation.setYawPitchRoll(yawPitchRoll);
      
      robot.setOrientation(frameOrientation.getQuaternion());
      robot.update();
   }
   
   private double getPelvisToFoot(SDFRobot robot)
   {
      List<GroundContactPoint> contactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      double height = Double.POSITIVE_INFINITY;
      for(GroundContactPoint gc : contactPoints)
      {
         if(gc.getPositionPoint().getZ() < height)
         {
            height = gc.getPositionPoint().getZ();
         }
      }
      return offset.getZ() - height;
   }
   
   public void getOffset(Vector3d offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3d offset)
   {
      this.offset.set(offset);
   }

   @Override
   public void setInitialYaw(double yaw)
   {
      initialYaw = yaw;
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   @Override
   public double getInitialYaw()
   {
      return initialYaw;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return groundZ;
   }
}
