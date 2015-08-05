package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;


public class UserDesiredHandPoseProvider implements HandPoseProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final EnumYoVariable<HandPosePacket.DataType> userHandPoseDataType = new EnumYoVariable<HandPosePacket.DataType>("userHandPoseDataType", registry,
                                                                                   HandPosePacket.DataType.class);
   private final DoubleYoVariable userHandPoseTrajectoryTime = new DoubleYoVariable("userHandPoseTrajectoryTime", registry);
   private final BooleanYoVariable userHandPoseTakeEm = new BooleanYoVariable("userHandPoseTakeEm", registry);

   private final EnumYoVariable<RobotSide> userHandPoseSide = new EnumYoVariable<RobotSide>("userHandPoseSide", registry, RobotSide.class);

   private final DoubleYoVariable userHandPoseX = new DoubleYoVariable("userHandPoseX", registry);
   private final DoubleYoVariable userHandPoseY = new DoubleYoVariable("userHandPoseY", registry);
   private final DoubleYoVariable userHandPoseZ = new DoubleYoVariable("userHandPoseZ", registry);

   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();
   private final SideDependentList<Map<OneDoFJoint, Double>> finalDesiredJointAngleMaps = new SideDependentList<Map<OneDoFJoint, Double>>();

   private final BooleanYoVariable[] yoUserHandControlledOrientationAxes = new BooleanYoVariable[3];
   private final boolean[] userHandControlledOrientationAxes = new boolean[SpatialMotionVector.SIZE];

   private final DoubleYoVariable userHandPercentOfTrajectoryWithOrientationControlled = new DoubleYoVariable("userHandPercentOfTrajectoryWithOrientationControlled", registry);

   private final ReferenceFrame chestFrame;

   private final SideDependentList<ReferenceFrame> packetReferenceFrames;
   private final FullRobotModel fullRobotModel;

   public UserDesiredHandPoseProvider(FullRobotModel fullRobotModel, SideDependentList<RigidBodyTransform> desiredHandPosesWithRespectToChestFrame, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      packetReferenceFrames = new SideDependentList<ReferenceFrame>(chestFrame, chestFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose homePose = new FramePose(chestFrame, desiredHandPosesWithRespectToChestFrame.get(robotSide));
         homePositions.put(robotSide, homePose);
         
         desiredHandPoses.put(robotSide, homePose);

         finalDesiredJointAngleMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, Double>());
      }

      yoUserHandControlledOrientationAxes[0] = new BooleanYoVariable("userHandControlAngularX", registry);
      yoUserHandControlledOrientationAxes[1] = new BooleanYoVariable("userHandControlAngularY", registry);
      yoUserHandControlledOrientationAxes[2] = new BooleanYoVariable("userHandControlAngularZ", registry);

      for (int i = 0; i < yoUserHandControlledOrientationAxes.length; i++)
         yoUserHandControlledOrientationAxes[i].set(true);

      userHandPercentOfTrajectoryWithOrientationControlled.set(1.0);
      userHandPoseTrajectoryTime.set(1.0);
      
      parentRegistry.addChild(registry);
   }

   private void updateFromNewestPacket(RobotSide robotSide)
   {
      FramePose userDesiredHandPose = new FramePose(homePositions.get(robotSide));
      FramePoint position = new FramePoint();
      userDesiredHandPose.getPositionIncludingFrame(position);

      position.setX(position.getX() + userHandPoseX.getDoubleValue());
      position.setY(position.getY() + userHandPoseY.getDoubleValue());
      position.setZ(position.getZ() + userHandPoseZ.getDoubleValue());

      userDesiredHandPose.setPosition(position);
      desiredHandPoses.put(robotSide, userDesiredHandPose);
      packetReferenceFrames.put(robotSide, userDesiredHandPose.getReferenceFrame());

//      System.out.println("userDesiredHandPose = " + userDesiredHandPose);

      Map<OneDoFJoint, Double> finalDesiredJointAngleMap = finalDesiredJointAngleMaps.get(robotSide);

      for (ArmJointName armJoint : fullRobotModel.getRobotSpecificJointNames().getArmJointNames())
      {
         finalDesiredJointAngleMap.put(fullRobotModel.getArmJoint(robotSide, armJoint), 0.0);
      }

   }

   @Override
   public void clear()
   {
      userHandPoseTakeEm.set(false);
   }

   @Override
   public boolean checkForNewPose(RobotSide robotSide)
   {
      if (userHandPoseSide.getEnumValue() != robotSide)
         return false;

      return userHandPoseTakeEm.getBooleanValue();
   }

   @Override
   public HandPosePacket.DataType checkHandPosePacketDataType(RobotSide robotSide)
   {
      return userHandPoseDataType.getEnumValue();
   }

   @Override
   public boolean checkForHomePosition(RobotSide robotSide)
   {
      return false; 
      
//      if (!checkForNewPose(robotSide))
//         return false;
//
////    if (!packets.get(robotSide).get().isToHomePosition())
////       return false;
//
//      desiredHandPoses.put(robotSide, homePositions.get(robotSide));
//
//      return true;
   }

   @Override
   public FramePose getDesiredHandPose(RobotSide robotSide)
   {
      updateFromNewestPacket(robotSide);

      userHandPoseTakeEm.set(false);
      return desiredHandPoses.get(robotSide);
   }

   @Override
   public Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide)
   {
      updateFromNewestPacket(robotSide);

      return finalDesiredJointAngleMaps.get(robotSide);
   }

   @Override
   public ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide)
   {
      return packetReferenceFrames.get(robotSide);
   }

   @Override
   public double getTrajectoryTime()
   {
      return userHandPoseTrajectoryTime.getDoubleValue();
   }

   @Override
   public boolean checkAndResetStopCommand(RobotSide robotSide)
   {
      return false;
   }

   @Override
   public boolean[] getControlledOrientationAxes(RobotSide robotSide)
   {
      for (int i = 0; i < userHandControlledOrientationAxes.length; i++)
         userHandControlledOrientationAxes[i] = yoUserHandControlledOrientationAxes[i].getBooleanValue();
      return userHandControlledOrientationAxes;
   }

   @Override
   public double getPercentOfTrajectoryWithOrientationBeingControlled(RobotSide robotSide)
   {
      return userHandPercentOfTrajectoryWithOrientationControlled.getDoubleValue();
   }

   @Override
   public boolean checkForNewPoseList(RobotSide robotSide)
   {
      return false;
   }

   @Override
   public Map<OneDoFJoint, double[]> getDesiredJointAngleForWaypointTrajectory(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public FramePose[] getDesiredHandPoses(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public DataType checkHandPoseListPacketDataType(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public boolean checkForNewRotateAboutAxisPacket(RobotSide robotSide)
   {
      return false;
   }

   @Override
   public Point3d getRotationAxisOriginInWorld(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public Vector3d getRotationAxisInWorld(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public double getRotationAngleRightHandRule(RobotSide robotSide)
   {
      return 0;
   }

   @Override
   public boolean checkForNewArmJointTrajectory(RobotSide robotSide)
   {
      return false;
   }

   @Override
   public ArmJointTrajectoryPacket getArmJointTrajectoryPacket(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public boolean controlHandAngleAboutAxis(RobotSide robotSide)
   {
      return false;
   }

   @Override
   public double getGraspOffsetFromControlFrame(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return 0.0;
   }

   @Override
   public Vector3d getForceConstraint(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public double getTangentialForce(RobotSide robotSide)
   {
      return Double.NaN;
   }

   @Override
   public HandRotateAboutAxisPacket.DataType checkHandRotateAboutAxisDataType(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }
}
