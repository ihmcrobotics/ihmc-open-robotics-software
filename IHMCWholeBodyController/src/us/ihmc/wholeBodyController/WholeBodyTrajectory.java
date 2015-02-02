package us.ihmc.wholeBodyController;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket.WholeBodyPose;
import us.ihmc.utilities.math.Vector64F;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.trajectory.TrajectoryND;
import us.ihmc.utilities.trajectory.TrajectoryND.WaypointND;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;


public class WholeBodyTrajectory
{
   static public TrajectoryND createJointSpaceTrajectory(
         final WholeBodyIkSolver wbSolver,  
         final OneDoFJoint[] initialState, 
         final OneDoFJoint[] finalState ) throws Exception
   {  
      int N = initialState.length;

      double[] deltaQ = new double[N];
      double[] wpQ = new double[N];

      double max = -1e10;

      for (int j=0; j<N; j++)
      {
         OneDoFJoint jointI = initialState[j];
         OneDoFJoint jointF = finalState[j];
         deltaQ[j] = jointF.getQ() - jointI.getQ();

         if(max < Math.abs(deltaQ[j])) {
            max = Math.abs(deltaQ[j]);
         }
      }

      double maxDeltaQ = 0.2;

      int segments = (int) Math.round(max/maxDeltaQ);

      TrajectoryND wb_trajectory = new TrajectoryND(N, 0.5, 10 );

      for (int s=0; s <= segments; s++ )
      {
         for (int j=0; j<N; j++)
         {
            OneDoFJoint jointI = initialState[j];
            wpQ[j] = jointI.getQ() + s*( deltaQ[j] / segments);
         }
         wb_trajectory.addWaypoint(wpQ);
      }

      wb_trajectory.buildTrajectory();
      return wb_trajectory;
   }


   static public TrajectoryND createTaskSpaceTrajectory(
         final WholeBodyIkSolver wbSolver,  
         final SDFFullRobotModel actualRobotModel,
         final SDFFullRobotModel finalRobotModel
         ) throws Exception
   {
      int N = wbSolver.getNumberOfJoints();

      HashMap<String,Integer> nameToIndex = new HashMap<String,Integer>();

      SideDependentList<ControlledDoF> previousOption = new SideDependentList<ControlledDoF>();
      SideDependentList<RigidBodyTransform> initialTransform  = new SideDependentList<RigidBodyTransform>();
      SideDependentList<RigidBodyTransform> finalTransform    = new SideDependentList<RigidBodyTransform>();

      for (RobotSide side: RobotSide.values)
      {
         ReferenceFrame initialTargetFrame = actualRobotModel.getHandControlFrame( side );
         ReferenceFrame finalTargetFrame   = finalRobotModel.getHandControlFrame( side );

         ReferenceFrame attachment = wbSolver.getDesiredGripperAttachmentFrame(side, ReferenceFrame.getWorldFrame() );
         ReferenceFrame palm       = wbSolver.getDesiredGripperPalmFrame(side, ReferenceFrame.getWorldFrame() );

         RigidBodyTransform attachmentToPalm = palm.getTransformToDesiredFrame( attachment );
         previousOption.set( side, wbSolver.getNumberOfControlledDoF(side) );

         RigidBodyTransform transform =  initialTargetFrame.getTransformToWorldFrame();
         transform.multiply( attachmentToPalm );
         initialTransform.set( side, transform );

         transform =  finalTargetFrame.getTransformToWorldFrame();
         transform.multiply( attachmentToPalm );
         finalTransform.set( side, transform );

         if( previousOption.get(side) != ControlledDoF.DOF_NONE)
         {
            wbSolver.setNumberOfControlledDoF(side, ControlledDoF.DOF_3P3R ); 
         }
      }

      HashMap<String, Double> anglesToUseAsInitialState = new HashMap<String, Double> ();
      HashMap<String, Double> outputAngles = new HashMap<String, Double> ();


      for (OneDoFJoint joint: actualRobotModel.getOneDoFJoints() )
      {
         if( wbSolver.hasJoint( joint.getName())  )
         {
            nameToIndex.put( joint.getName(), nameToIndex.size() );
         }    
      }

      int segmentsPos = 1;
      int segmentsRot = 1;

      double maxDeltaPos = 0.1;
      double maxDeltaRot = 0.3;

      for (RobotSide side: RobotSide.values)
      {
         ControlledDoF numberOfDoF = wbSolver.getNumberOfControlledDoF(side);

         if( numberOfDoF != ControlledDoF.DOF_NONE )
         {
            double distance = RigidBodyTransform.getTranslationDifference(
                  initialTransform.get(side), 
                  finalTransform.get(side) ).length();

            segmentsPos = (int) Math.max(segmentsPos, Math.round( distance / maxDeltaPos) );
         }
         if( numberOfDoF == ControlledDoF.DOF_3P2R || numberOfDoF ==  ControlledDoF.DOF_3P3R )
         {
            double distance = RigidBodyTransform.getRotationDifference(
                  initialTransform.get(side), 
                  finalTransform.get(side) ).length();

            segmentsRot = (int)  Math.max(segmentsPos, Math.round( distance/ maxDeltaRot) );
         }
      }

      int numSegments = Math.max(segmentsRot, segmentsPos);

      Vector64F thisWaypointAngles = new Vector64F(N);

      for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
      {
         String jointName = entry.getKey();
         int index = entry.getValue();

         double initialAngle = actualRobotModel.getOneDoFJointByName(jointName).getQ();
         thisWaypointAngles.set( index, initialAngle );   
      }


      TrajectoryND wb_trajectory = new TrajectoryND(N, 0.5, 10 );

      System.out.println("---------------- ");

      for (int s=0; s <= numSegments; s++ )
      {
         for (RobotSide side: RobotSide.values)
         {
            RigidBodyTransform interpolatedTransform = new RigidBodyTransform();

            double alpha = ((double)s)/numSegments;
            interpolatedTransform.interpolate(initialTransform.get(side), finalTransform.get(side), alpha);

            FramePose target =  new FramePose( ReferenceFrame.getWorldFrame(), interpolatedTransform );

            wbSolver.setGripperPalmTarget( actualRobotModel,  side, target );

            if( side == RobotSide.RIGHT )
            {
               RigidBodyTransform targetTrans = new RigidBodyTransform();
               target.getRigidBodyTransform( targetTrans );
               System.out.format( "%s: %d/%d / %.2f :\n%s\n", side, s, numSegments, alpha, targetTrans );
            }
         }

         if( s > 0 /* && s!= numSegments */ )
         {
            for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
            {
               String jointName = entry.getKey();
               int index = entry.getValue();

               double currentAngle = thisWaypointAngles.get(index);
               double finalAngle   = finalRobotModel.getOneDoFJointByName(jointName).getQ();  
               double alpha = 1.0 / (double) ( 1  + numSegments - s  );
              if( alpha < 1.0) alpha = 0;

               thisWaypointAngles.set(index, currentAngle*(1.0 -alpha) + finalAngle*alpha );   
            }

            for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
            {
               final String jointName = entry.getKey();
               final int index = entry.getValue();        
               anglesToUseAsInitialState.put( jointName, thisWaypointAngles.get(index) );
            }

            if( s!= numSegments )
            {
               //---------------------------------
               int ret =  wbSolver.compute(actualRobotModel,  anglesToUseAsInitialState, outputAngles);
               //---------------------------------

               if( ret > -2)
               {
                  for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
                  {
                     final String jointName = entry.getKey();
                     final int index = entry.getValue();  
                     Double angle = outputAngles.get( jointName );
                     thisWaypointAngles.set(index, angle ); 
                  }
               }
            }
         }
         wb_trajectory.addWaypoint(thisWaypointAngles.data);
      }

      for (RobotSide side: RobotSide.values)
      {
         wbSolver.setNumberOfControlledDoF(side, previousOption.get(side) ); 
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
      int numWaypoints = 0;//wbTrajectory.getNumWasypoints();
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
