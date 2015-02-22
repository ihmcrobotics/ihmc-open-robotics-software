package us.ihmc.wholeBodyController;

import java.util.ArrayList;
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
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.trajectory.TrajectoryND;
import us.ihmc.utilities.trajectory.TrajectoryND.WaypointND;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.LockLevel;


public class WholeBodyTrajectory
{
   private double maxJointVelocity;
   private double maxJointAcceleration;
   private double maxDistanceInTaskSpaceBetweenWaypoints;

   public WholeBodyTrajectory(double maxJointVelocity, double maxJointAcceleration, double maxDistanceInTaskSpaceBetweenWaypoints)
   {
      this.maxJointVelocity = maxJointVelocity;
      this.maxJointAcceleration = maxJointAcceleration;
      this.maxDistanceInTaskSpaceBetweenWaypoints = maxDistanceInTaskSpaceBetweenWaypoints;
   }

   public TrajectoryND createTaskSpaceTrajectory(
         final WholeBodyIkSolver wbSolver,  
         final SDFFullRobotModel initialRobotState,
         final SDFFullRobotModel finalRoboState
         ) throws Exception
   {
      int N = wbSolver.getNumberOfJoints();

     // maxDistanceInTaskSpaceBetweenWaypoints = 0.4;

      SDFFullRobotModel currentRobotModel = new SDFFullRobotModel( initialRobotState );
      InverseDynamicsJointStateCopier copier = new InverseDynamicsJointStateCopier(
            initialRobotState.getElevator(), currentRobotModel.getElevator() );
      
      copier.copy();
      
      int oldMaxReseed = wbSolver.maxNumberOfAutomaticReseeds;
      wbSolver.maxNumberOfAutomaticReseeds = 0;

      HashMap<String,Integer> nameToIndex = new HashMap<String,Integer>();

      SideDependentList<ControlledDoF> previousOption = new SideDependentList<ControlledDoF>();
      SideDependentList<RigidBodyTransform> initialTransform  = new SideDependentList<RigidBodyTransform>();
      SideDependentList<RigidBodyTransform> finalTransform    = new SideDependentList<RigidBodyTransform>();
      LockLevel previousLockLimit = wbSolver.getLockLevel();
      
      wbSolver.setLockLevel( LockLevel.LOCK_LEGS_AND_WAIST );

      for (RobotSide side: RobotSide.values)
      {
         ReferenceFrame initialTargetFrame = initialRobotState.getHandControlFrame( side );
         ReferenceFrame finalTargetFrame   = finalRoboState.getHandControlFrame( side );

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


      HashMap<String, Double> outputAngles = new HashMap<String, Double> ();


      ArrayList<String> jointNames = new ArrayList<String>();

      for (OneDoFJoint joint: initialRobotState.getOneDoFJoints() )
      {
         if( wbSolver.hasJoint( joint.getName())  )
         {
            nameToIndex.put( joint.getName(), nameToIndex.size() );
            jointNames.add( joint.getName() );
         }    
      }

      int segmentsPos = 1;
      int segmentsRot = 1;

      double maxDeltaPos = maxDistanceInTaskSpaceBetweenWaypoints;
      double maxDeltaRot = maxDistanceInTaskSpaceBetweenWaypoints;

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

      //numSegments = 4;

      Vector64F thisWaypointAngles = new Vector64F(N);
      HashMap<String,Double> thisWaypointAnglesByName = new HashMap<String,Double>();

      for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
      {
         String jointName = entry.getKey();
         double initialAngle = initialRobotState.getOneDoFJointByName(jointName).getQ();
         thisWaypointAngles.set( entry.getValue(), initialAngle );  
         thisWaypointAnglesByName.put( jointName, initialAngle );   
      }

      TrajectoryND wb_trajectory = new TrajectoryND(N, maxJointVelocity,  maxJointAcceleration );

      wb_trajectory.addNames( jointNames );

      for (int s=0; s <= numSegments; s++ )
      {

         if( s > 0  )
         {
            for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
            {
               String jointName = entry.getKey();
               int index = entry.getValue();

               double alpha = ((double)s) / (double) ( numSegments  ); 

               double initialAngle = initialRobotState.getOneDoFJointByName(jointName).getQ();
               double finalAngle   = finalRoboState.getOneDoFJointByName(jointName).getQ(); 
               double currentAngle = currentRobotModel.getOneDoFJointByName(jointName).getQ(); 
               
               //double interpolatedAngle = currentAngle*(1.0 -alpha) + finalAngle*alpha ;
               double interpolatedAngle = initialAngle*(1.0 -alpha) + finalAngle*alpha ;
               
               thisWaypointAngles.set(index,interpolatedAngle );
               thisWaypointAnglesByName.put( jointName, interpolatedAngle  );
            }
  
            currentRobotModel.updateJointsAngleButKeepOneFootFixed(thisWaypointAnglesByName, RobotSide.RIGHT);
            
            for (RobotSide side: RobotSide.values)
            {
               RigidBodyTransform interpolatedTransform = new RigidBodyTransform();

               double alpha = ((double)s)/(numSegments);
               interpolatedTransform.interpolate(initialTransform.get(side), finalTransform.get(side), alpha);

               FramePose target =  new FramePose( ReferenceFrame.getWorldFrame(), interpolatedTransform );
               
               wbSolver.setGripperPalmTarget( initialRobotState,  side, target );

               if( side == RobotSide.RIGHT )
               {
                  RigidBodyTransform targetTrans = new RigidBodyTransform();
                  target.getRigidBodyTransform( targetTrans );
               }
            }

            if( s < numSegments )
            {
               //---------------------------------
                ComputeResult ret =  wbSolver.compute(currentRobotModel,  thisWaypointAnglesByName, outputAngles);
               //---------------------------------

               // note: use also the failed one that didn't converge... better than nothing.
               if( ret != ComputeResult.FAILED_INVALID)
               {
                  for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
                  {
                     String jointName = entry.getKey();
                     int index = entry.getValue();
                     Double angle = outputAngles.get( jointName );
                     thisWaypointAngles.set(index, angle );
                     thisWaypointAnglesByName.put( jointName, angle );  
                  }
               }
               else{
                  System.out.println("OOOOPS");
                  for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
                  {
                     
                     String jointName = entry.getKey();
                     int index = entry.getValue();
                     double alpha = 1.0 / (double) ( numSegments + 1 -s ); 
   
                     double currentAngle = thisWaypointAnglesByName.get(jointName);
                     double finalAngle   = finalRoboState.getOneDoFJointByName(jointName).getQ();  
                     
                     double interpolatedAngle = currentAngle*(1.0 -alpha) + finalAngle*alpha ;
                     thisWaypointAngles.set(index, interpolatedAngle );
                     thisWaypointAnglesByName.put( jointName, interpolatedAngle );  
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

      wbSolver.maxNumberOfAutomaticReseeds = oldMaxReseed;
      
      wbSolver.setLockLevel( previousLockLimit );
      
      return wb_trajectory;
   }


   static public WholeBodyTrajectoryPacket convertTrajectoryToPacket(
         SDFFullRobotModel model,
         TrajectoryND wbTrajectory)
   {
      // build the trajectory packet from the new waypoints

      int numJointsPerArm = model.armJointIDsList.get(RobotSide.LEFT).size();
      int numWaypoints = wbTrajectory.getNumWaypoints();
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
