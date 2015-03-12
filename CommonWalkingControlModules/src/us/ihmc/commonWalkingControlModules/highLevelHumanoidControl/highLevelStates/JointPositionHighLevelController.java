package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.HashMap;
import java.util.HashSet;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class JointPositionHighLevelController extends HighLevelBehavior
{

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HashSet<OneDoFJoint> jointsBeenControlled = new HashSet<OneDoFJoint>();
   private final FullRobotModel fullRobotModel;

   public final static HighLevelState controllerState = HighLevelState.JOINT_PID_CONTROL;

   private final HashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> trajectoryGenerator; 
   private final YoVariableDoubleProvider trajectoryTimeProvider;
   private final DoubleYoVariable timeProvider;

   private double initialTrajectoryTime;
   private final DesiredJointsPositionProvider desiredJointsProvider;

   
   public JointPositionHighLevelController(MomentumBasedController momentumBasedController, DesiredJointsPositionProvider desiredJointsProvider)
   {
      super(controllerState);
      
      timeProvider = momentumBasedController.getYoTime();
      this.desiredJointsProvider = desiredJointsProvider;

      trajectoryGenerator = new HashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();
      
      fullRobotModel = momentumBasedController.getFullRobotModel();
      trajectoryTimeProvider = new YoVariableDoubleProvider("jointControl_trajectory_time", registry);
      
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         String joinName = joint.getName();

         if( joinName.contains("finger") ) continue;
         if( joinName.contains("hokuyo") ) continue;
         if( joinName.contains("neck") )   continue;

         jointsBeenControlled.add(joint);
         
         OneDoFJointQuinticTrajectoryGenerator generator = new OneDoFJointQuinticTrajectoryGenerator("jointControl_"+ joint.getName() , joint, trajectoryTimeProvider, registry);
         trajectoryGenerator.put(joint,  generator );
      }
   }
   
   private void initializeFromPacket(JointAnglesPacket packet)
   {
      initialTrajectoryTime = timeProvider.getDoubleValue();
      System.out.println("initialTrajectoryTime " + initialTrajectoryTime);
      trajectoryTimeProvider.set( packet.trajectoryTime );
    //  ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      
    //  OneDoFJoint[] spineJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getChest()), OneDoFJoint.class);
      
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.LEFT, ArmJointName.SHOULDER_YAW)   ).setFinalPosition(packet.leftArmJointAngle[0] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.LEFT, ArmJointName.SHOULDER_ROLL)  ).setFinalPosition( packet.leftArmJointAngle[1] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.LEFT, ArmJointName.ELBOW_PITCH)    ).setFinalPosition( packet.leftArmJointAngle[2] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.LEFT, ArmJointName.ELBOW_ROLL)     ).setFinalPosition( packet.leftArmJointAngle[3] );             
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.LEFT, ArmJointName.WRIST_PITCH)    ).setFinalPosition( packet.leftArmJointAngle[4] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.LEFT, ArmJointName.WRIST_ROLL)     ).setFinalPosition( packet.leftArmJointAngle[5] );
                    
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.RIGHT, ArmJointName.SHOULDER_YAW) ).setFinalPosition(   packet.rightArmJointAngle[0] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL)  ).setFinalPosition( packet.rightArmJointAngle[1] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.RIGHT, ArmJointName.ELBOW_PITCH)    ).setFinalPosition( packet.rightArmJointAngle[2] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.RIGHT, ArmJointName.ELBOW_ROLL)     ).setFinalPosition( packet.rightArmJointAngle[3] );             
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.RIGHT, ArmJointName.WRIST_PITCH)    ).setFinalPosition( packet.rightArmJointAngle[4] );
      trajectoryGenerator.get( fullRobotModel.getArmJoint( RobotSide.RIGHT, ArmJointName.WRIST_ROLL)     ).setFinalPosition( packet.rightArmJointAngle[5] );

      trajectoryGenerator.get( fullRobotModel.getSpineJoint( SpineJointName.SPINE_PITCH) ).setFinalPosition( packet.waistJointAngle[0] );
      trajectoryGenerator.get( fullRobotModel.getSpineJoint( SpineJointName.SPINE_ROLL)  ).setFinalPosition( packet.waistJointAngle[1] );
      trajectoryGenerator.get( fullRobotModel.getSpineJoint( SpineJointName.SPINE_YAW)   ).setFinalPosition( packet.waistJointAngle[2] );

      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.LEFT, LegJointName.HIP_YAW)    ).setFinalPosition( packet.leftLegJointAngle[0] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.LEFT, LegJointName.HIP_ROLL)   ).setFinalPosition( packet.leftLegJointAngle[1] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.LEFT, LegJointName.HIP_PITCH)  ).setFinalPosition( packet.leftLegJointAngle[2] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.LEFT, LegJointName.KNEE)       ).setFinalPosition( packet.leftLegJointAngle[3] );             
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.LEFT, LegJointName.ANKLE_PITCH)).setFinalPosition( packet.leftLegJointAngle[4] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.LEFT, LegJointName.ANKLE_ROLL) ).setFinalPosition( packet.leftLegJointAngle[5] );
      
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.RIGHT, LegJointName.HIP_YAW)    ).setFinalPosition( packet.rightLegJointAngle[0] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.RIGHT, LegJointName.HIP_ROLL)   ).setFinalPosition( packet.rightLegJointAngle[1] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.RIGHT, LegJointName.HIP_PITCH)  ).setFinalPosition( packet.rightLegJointAngle[2] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.RIGHT, LegJointName.KNEE)       ).setFinalPosition( packet.rightLegJointAngle[3] );             
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.RIGHT, LegJointName.ANKLE_PITCH)).setFinalPosition( packet.rightLegJointAngle[4] );
      trajectoryGenerator.get( fullRobotModel.getLegJoint( RobotSide.RIGHT, LegJointName.ANKLE_ROLL) ).setFinalPosition( packet.rightLegJointAngle[5] );
      
      for (OneDoFJoint joint: jointsBeenControlled)
      {
         trajectoryGenerator.get(joint).initialize();
      } 
   }
   

   @Override
   public void doAction()
   {

      if(desiredJointsProvider != null && desiredJointsProvider.checkForNewPacket() )
      {
         initializeFromPacket( desiredJointsProvider.getNewPacket() );
      }

      double time = timeProvider.getDoubleValue() - initialTrajectoryTime;
     // System.out.println(" time " + time); 
      
      for (OneDoFJoint joint: jointsBeenControlled)
      {   
         OneDoFJointQuinticTrajectoryGenerator generator = trajectoryGenerator.get(joint);
         
         if( generator.isDone() == false)
         {
            generator.compute(time);
         //   System.out.println("compute " + time + " / " +  trajectoryTimeProvider.getValue() );
         }
         joint.setUnderPositionControl(true);
         joint.setqDesired( generator.getValue() );
         joint.setQdDesired(  generator.getVelocity() );
      } 
   }

   @Override
   public void doTransitionIntoAction()
   {      
      for (OneDoFJoint joint: jointsBeenControlled)
      {
         trajectoryGenerator.get(joint).setFinalPosition(joint.getQ() );
         trajectoryGenerator.get(joint).initialize(joint.getQ(), joint.getQd() );
      } 
   }

   @Override
   public void doTransitionOutOfAction()
   {      
   }


   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

}
