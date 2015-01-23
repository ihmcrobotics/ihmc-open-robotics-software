package us.ihmc.wholeBodyController;

import java.util.HashMap;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket.WholeBodyPose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.trajectory.TrajectoryND;
import us.ihmc.utilities.trajectory.TrajectoryND.WaypointND;


public class WholeBodyTrajectory
{
   static public TrajectoryND createWholeBodyTrajectory(
         final WholeBodyIkSolver wbSolver,  
         final SDFFullRobotModel initialState, 
         final SDFFullRobotModel finalState ) throws Exception
   {
      int N = initialState.getOneDoFJoints().length;

      double[] deltaQ = new double[N];
      double[] wpQ = new double[N];

      double max = -1e10;

      HashMap<String, Double> anglesToUseAsInitialState = new HashMap<String, Double> ();
      HashMap<String, Double> outputAngles = new HashMap<String, Double> ();

      for (int j=0; j<N; j++)
      {
         OneDoFJoint jointI = initialState.getOneDoFJoints()[j];
         OneDoFJoint jointF = finalState.getOneDoFJoints()[j];
         deltaQ[j] = jointF.getQ() - jointI.getQ();

         if(max < Math.abs(deltaQ[j])) {
            max = Math.abs(deltaQ[j]);
         }
      }

      double maxDeltaQ = 0.3;

      int segments = (int) Math.round(max/maxDeltaQ);

      TrajectoryND wb_trajectory = new TrajectoryND(N, 0.5, 10 );

      for (int s=0; s <= segments; s++ )
      {
         for (int j=0; j<N; j++)
         {
            OneDoFJoint jointI = initialState.getOneDoFJoints()[j];
            wpQ[j] = jointI.getQ() + s*( deltaQ[j] / segments);
         }

         if( s > 0 && s < segments )
         {
            for (int j=0; j<N; j++)
            {
               String jointName = initialState.getOneDoFJoints()[j].getName();
               anglesToUseAsInitialState.put( jointName, new Double( wpQ[j] ) );
            }

            boolean jointPoseEnabled     = wbSolver.task_joints_pose.isEnabled();
            boolean leftPositionEnabled  = wbSolver.task_end_effector_translations.get(RobotSide.LEFT).isEnabled();
            boolean leftRotationEnabled  = wbSolver.task_end_effector_rotations.get(RobotSide.LEFT).isEnabled();
            boolean rightPositionEnabled = wbSolver.task_end_effector_translations.get(RobotSide.RIGHT).isEnabled();
            boolean rightRotationEnabled = wbSolver.task_end_effector_rotations.get(RobotSide.RIGHT).isEnabled();

            wbSolver.task_joints_pose.setEnabled(false);
            wbSolver.task_end_effector_translations.get(RobotSide.LEFT).setEnabled(false);
            wbSolver.task_end_effector_rotations.get(RobotSide.LEFT).setEnabled(false);
            wbSolver.task_end_effector_translations.get(RobotSide.RIGHT).setEnabled(false);
            wbSolver.task_end_effector_rotations.get(RobotSide.RIGHT).setEnabled(false);

            wbSolver.compute(anglesToUseAsInitialState, outputAngles);

            wbSolver.task_joints_pose.setEnabled(jointPoseEnabled);
            wbSolver.task_end_effector_translations.get(RobotSide.LEFT).setEnabled(leftPositionEnabled);
            wbSolver.task_end_effector_rotations.get(RobotSide.LEFT).setEnabled(leftRotationEnabled);
            wbSolver.task_end_effector_translations.get(RobotSide.RIGHT).setEnabled(rightPositionEnabled);
            wbSolver.task_end_effector_rotations.get(RobotSide.RIGHT).setEnabled(rightRotationEnabled);

            for (int j=0; j<N; j++)
            {
               String jointName = initialState.getOneDoFJoints()[j].getName();
               Double value = outputAngles.get( jointName);
               if( value != null) {
                  wpQ[j] = value.doubleValue();
               }
            }
         }
         wb_trajectory.addWaypoint(wpQ);
      }

      wb_trajectory.buildTrajectory();

      return wb_trajectory;
   }



   static public WholeBodyTrajectoryPacket convertTrajectoryToPacket(
         SDFFullRobotModel model,
         TrajectoryND wbTrajectory)
   {
      // build the trajectory packet from the new waypoints

      int numJointsPerArm = model.armJointIDsList.get(RobotSide.LEFT).size();
      int numWaypoints = wbTrajectory.getNumWasypoints();
      int numJoints = wbTrajectory.getNumDimensions();


      ReferenceFrame fixedFoot = model.getSoleFrame(RobotSide.RIGHT);

      WholeBodyTrajectoryPacket packet = new WholeBodyTrajectoryPacket(numWaypoints,numJointsPerArm);

      OneDoFJoint[] oneDoFJoints = model.getOneDoFJoints();

      for (int w=0; w<numWaypoints; w++ )
      {
         WaypointND    jointsWaypoint = wbTrajectory.getWaypoint(w);
         WholeBodyPose packetWaypoint = packet.waypoints[w];

         //just take the first one, they are suppose to be all the same.
         packetWaypoint.absTime = jointsWaypoint.absTime;

         int J = 0;
         // TODO: check the order.

         for (RobotSide side: RobotSide.values)
         {
            for(OneDoFJoint joint: model.armJointIDsList.get(side) )
            {
               String jointName = joint.getName();
               int index = -1;
               for(index=0; index< numJoints; index++ )
               {
                  if( oneDoFJoints[index].getName().equals(jointName) ) break;
               } 
               if( side.equals( RobotSide.LEFT))
               {
                  packetWaypoint.leftArmJointAngle[J] = jointsWaypoint.position[index];
                  packetWaypoint.leftArmJointAngle[J] = jointsWaypoint.velocity[index];
               }
               else{
                  packetWaypoint.rightArmJointAngle[J]    = jointsWaypoint.position[index];
                  packetWaypoint.rightArmJointVelocity[J] = jointsWaypoint.velocity[index];
               }
               J++;
            }
         }      

         RigidBodyTransform[] worldToPelvis = new RigidBodyTransform[2];
         RigidBodyTransform[] worldToChest = new RigidBodyTransform[2];

         final double dT = 0.0001;
         
         for (int i=0; i< 2; i++)
         {
            worldToPelvis[i] = new RigidBodyTransform();
            worldToChest[i]  = new RigidBodyTransform();

            for (int j=0; j<numJoints; j++  )
            {
               double jointAngle =  jointsWaypoint.position[j];
               
               if( i==1 )
               {
                  jointAngle += dT*jointsWaypoint.velocity[j];
               }
               model.getOneDoFJoints()[j].setQ( jointAngle );
            }
            model.updateFrames();

            ReferenceFrame pelvis    = model.getRootJoint().getFrameAfterJoint();
            ReferenceFrame movedFoot = model.getSoleFrame(RobotSide.RIGHT);
            ReferenceFrame chest     = model.getChest().getBodyFixedFrame();

            RigidBodyTransform worldToFixed = fixedFoot.getTransformToWorldFrame();
            RigidBodyTransform movedToPelvis = pelvis.getTransformToDesiredFrame(movedFoot);

            worldToPelvis[i].multiply( worldToFixed, movedToPelvis);

            RigidBodyTransform pelvisToChest = chest.getTransformToDesiredFrame( pelvis );
            worldToChest[i].multiply( worldToPelvis[i], pelvisToChest);
         }
         
         Vector3d positionA = new Vector3d();
         Vector3d positionB = new Vector3d();
         
         worldToPelvis[0].getTranslation(positionA);
         worldToPelvis[1].getTranslation(positionB);
         
         packetWaypoint.pelvisVelocity.x = (positionB.x - positionA.x)/dT;
         packetWaypoint.pelvisVelocity.y = (positionB.y - positionA.y)/dT;
         packetWaypoint.pelvisVelocity.z = (positionB.z - positionA.z)/dT;

         worldToPelvis[0].get( packetWaypoint.pelvisOrientation,  packetWaypoint.pelvisPosition);
         worldToChest[0].get( packetWaypoint.chestOrientation );
 
      }
      return null;
   }
}
