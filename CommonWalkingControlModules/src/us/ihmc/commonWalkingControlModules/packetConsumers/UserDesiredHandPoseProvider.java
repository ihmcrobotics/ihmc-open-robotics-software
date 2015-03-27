package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
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
         
//         System.out.println("homePose = " + homePose);

         desiredHandPoses.put(robotSide, homePose);

         finalDesiredJointAngleMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, Double>());
      }

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
   public boolean checkForNewPauseCommand(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void getPauseCommand(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      
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
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public DataType checkHandPoseListPacketDataType(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean checkForNewRotateAboutAxisPacket(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public Point3d getRotationAxisOriginInWorld(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public Vector3d getRotationAxisInWorld(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public double getRotationAngleRightHandRule(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public boolean checkForNewArmJointTrajectory(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public ArmJointTrajectoryPacket getArmJointTrajectoryPacket(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

}
