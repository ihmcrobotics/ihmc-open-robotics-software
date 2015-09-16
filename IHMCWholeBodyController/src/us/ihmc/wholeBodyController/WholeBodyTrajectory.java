package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.trajectories.TrajectoryND;
import us.ihmc.robotics.trajectories.TrajectoryND.WaypointND;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.LockLevel;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.WholeBodyConfiguration;


public class WholeBodyTrajectory
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private double maxJointVelocity;
   private double maxJointAcceleration;
   private double maxDistanceInTaskSpaceBetweenWaypoints;
   private final SDFFullHumanoidRobotModel currentRobotModel;
   private final HashMap<String,Integer> jointNameToTrajectoryIndex = new HashMap<String,Integer>();
   private final HashMap<String, Double> desiredJointAngles = new HashMap<String, Double> ();
   private RigidBodyTransform worldToFoot; 
   private final SDFFullHumanoidRobotModel fullRobotModel;
   private double minimumExecutionTime = 0.5;

   public WholeBodyTrajectory(SDFFullHumanoidRobotModel fullRobotModel, double maxJointVelocity, double maxJointAcceleration, double maxDistanceInTaskSpaceBetweenWaypoints)
   {
      currentRobotModel = new SDFFullHumanoidRobotModel( fullRobotModel );
      this.fullRobotModel = fullRobotModel;

      this.maxJointVelocity = maxJointVelocity;
      this.maxJointAcceleration = maxJointAcceleration;
      this.maxDistanceInTaskSpaceBetweenWaypoints = maxDistanceInTaskSpaceBetweenWaypoints;
   }
   
   public void setMinimumExecutionTime(double minTime)
   {
      minimumExecutionTime = minTime;
   }
   
   public double getMinimumExecutionTime()
   {
      return minimumExecutionTime;
   }

   public TrajectoryND createTaskSpaceTrajectory(
         final WholeBodyIkSolver wbSolver,  
         final SDFFullHumanoidRobotModel initialRobotState,
         final SDFFullHumanoidRobotModel finalRoboState
         ) throws Exception
   {
      int N = wbSolver.getNumberOfJoints();
      
      WholeBodyConfiguration savedParameters =  wbSolver.cloneConfiguration( );

      worldToFoot = initialRobotState.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();

      if( ! worldToFoot.epsilonEquals( finalRoboState.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame() , 0.001) )
      {
         System.out.println("WholeBodyTrajectory: potential error. Check the root foot (RIGHT) pose of initial and final models");
      }


      InverseDynamicsJointStateCopier copier = new InverseDynamicsJointStateCopier(
            initialRobotState.getElevator(), currentRobotModel.getElevator() );

      copier.copy();
      currentRobotModel.updateFrames();
      

      wbSolver.getConfiguration().setMaxNumberOfAutomaticReseeds(0);  

      SideDependentList<RigidBodyTransform> initialTransform  = new SideDependentList<RigidBodyTransform>();
      SideDependentList<RigidBodyTransform> finalTransform    = new SideDependentList<RigidBodyTransform>();

      wbSolver.getConfiguration().setLockLevel( LockLevel.LOCK_LEGS_AND_WAIST );

      for (RobotSide side: RobotSide.values)
      {
         ReferenceFrame initialTargetFrame = initialRobotState.getHandControlFrame( side );
         ReferenceFrame finalTargetFrame   = finalRoboState.getHandControlFrame( side );

         RigidBodyTransform transform =  initialTargetFrame.getTransformToWorldFrame();
         initialTransform.set( side, transform );

         transform =  finalTargetFrame.getTransformToWorldFrame();
         finalTransform.set( side, transform );

         if( wbSolver.getConfiguration().getNumberOfControlledDoF(side) != ControlledDoF.DOF_NONE)
         {
            wbSolver.getConfiguration().setNumberOfControlledDoF(side, ControlledDoF.DOF_3P2R ); 
         }
      }

      ArrayList<String> jointNames = new ArrayList<String>();

      int counter = 0;
      for (OneDoFJoint joint: initialRobotState.getOneDoFJoints() )
      {
         if( wbSolver.hasJoint( joint.getName())  )
         {
            jointNameToTrajectoryIndex.put( joint.getName(),counter);
            counter++;

            jointNames.add( joint.getName() );
         }    
      }

    /*  int segmentsPos = 1;
      int segmentsRot = 1;

      double maxDeltaPos = maxDistanceInTaskSpaceBetweenWaypoints;

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
      }
*/

      int numSegments = 1;

      Vector64F thisWaypointAngles = new Vector64F(N);
      HashMap<String,Double> thisWaypointAnglesByName = new HashMap<String,Double>();

      for ( Map.Entry<String, Integer> entry: jointNameToTrajectoryIndex.entrySet() )
      {
         String jointName = entry.getKey();
         int index = entry.getValue();
         double initialAngle = initialRobotState.getOneDoFJointByName(jointName).getQ();
         thisWaypointAngles.set( index, initialAngle );  
         thisWaypointAnglesByName.put( jointName, initialAngle );   
      }

      TrajectoryND wb_trajectory = new TrajectoryND(N, maxJointVelocity,  maxJointAcceleration );

      wb_trajectory.addNames( jointNames );

      for (int s=0; s <= numSegments; s++ )
      {
         if( s > 0  )
         {
            for ( Map.Entry<String, Integer> entry: jointNameToTrajectoryIndex.entrySet() )
            {
               String jointName = entry.getKey();
               int index = entry.getValue();

               double alpha = ((double)s) / (double) ( numSegments  ); 

               double initialAngle = initialRobotState.getOneDoFJointByName(jointName).getQ();
               double finalAngle   = finalRoboState.getOneDoFJointByName(jointName).getQ(); 
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

               FramePose target =  new FramePose( worldFrame, interpolatedTransform );

               wbSolver.setGripperPalmTarget( side, target );
            }

            if( s < numSegments )
            {
               //---------------------------------
               ComputeResult ret =  wbSolver.compute(currentRobotModel,  thisWaypointAnglesByName, desiredJointAngles);
               //---------------------------------

               // note: use also the failed one that didn't converge... better than nothing.
               // if( ret != ComputeResult.FAILED_INVALID)
               if( ret == ComputeResult.SUCCEEDED)
               {
                  //  System.out.println(" SUCCEEDED ");
                  for ( Map.Entry<String, Integer> entry: jointNameToTrajectoryIndex.entrySet() )
                  {
                     String jointName = entry.getKey();
                     int index = entry.getValue();
                     Double angle = desiredJointAngles.get( jointName );
                     thisWaypointAngles.set(index, angle );
                     thisWaypointAnglesByName.put( jointName, angle );  
                  }
                  wb_trajectory.addWaypoint(thisWaypointAngles.data);
               }
               else{
                  System.out.println("OOOOPS");
                   /*  for ( Map.Entry<String, Integer> entry: nameToIndex.entrySet() )
                  {
                     String jointName = entry.getKey();
                     int index = entry.getValue();
                     double alpha = 1.0 / (double) ( numSegments + 1 -s ); 

                     double currentAngle = thisWaypointAnglesByName.get(jointName);
                     double finalAngle   = finalRoboState.getOneDoFJointByName(jointName).getQ();  

                     double interpolatedAngle = currentAngle*(1.0 -alpha) + finalAngle*alpha ;
                     thisWaypointAngles.set(index, interpolatedAngle );
                     thisWaypointAnglesByName.put( jointName, interpolatedAngle );  
                  }*/
               }
            }
         }
         if( s==0 || s==numSegments )
         {
            wb_trajectory.addWaypoint(thisWaypointAngles.data);
         }
      }


      wb_trajectory.buildTrajectory( minimumExecutionTime);


      wbSolver.setConfiguration( savedParameters );

      // cleanup currentRobotModel
      copier.copy();
      currentRobotModel.updateFrames();

      return wb_trajectory;
   }


   public WholeBodyTrajectoryPacket convertTrajectoryToPacket(
         TrajectoryND wbTrajectory)
   {
      int numJointsPerArm = currentRobotModel.getArmJointIDs(RobotSide.LEFT).size();
      int numWaypoints    = wbTrajectory.getNumWaypoints();

      WholeBodyTrajectoryPacket packet = new WholeBodyTrajectoryPacket(numWaypoints ,numJointsPerArm);

      Vector3d temp = new Vector3d();

      for (int w=0; w < numWaypoints; w++ )
      {
         WaypointND    jointsWaypoint = wbTrajectory.getWaypoint(w);
                
         packet.timeAtWaypoint[w] = jointsWaypoint.absTime + 0.2;

         //-----  check if this is part of the arms ---------

         int jointIdx = 0 ;
         for(OneDoFJoint armJoint: currentRobotModel.getArmJointIDs( RobotSide.LEFT) )
         {   
            int index = jointNameToTrajectoryIndex.get( armJoint.getName() );
            packet.leftArmTrajectory.trajectoryPoints[w].positions[jointIdx] = jointsWaypoint.position[index];
            packet.leftArmTrajectory.trajectoryPoints[w].velocities[jointIdx] = jointsWaypoint.velocity[index];
            jointIdx++;
         }
         packet.leftArmTrajectory.trajectoryPoints[w].time = packet.timeAtWaypoint[w];

         jointIdx = 0 ;
         for(OneDoFJoint armJoint: currentRobotModel.getArmJointIDs( RobotSide.RIGHT) )
         {   
            int index = jointNameToTrajectoryIndex.get( armJoint.getName() );
            packet.rightArmTrajectory.trajectoryPoints[w].positions[jointIdx] = jointsWaypoint.position[index];
            packet.rightArmTrajectory.trajectoryPoints[w].velocities[jointIdx] = jointsWaypoint.velocity[index];
            jointIdx++;
         }
         packet.rightArmTrajectory.trajectoryPoints[w].time = packet.timeAtWaypoint[w];

         ///---------- calculation in task space ------------

         // store this before updating the model.

         // first, we need to copy position and velocities in the SDFFullRobotModel
         for ( Map.Entry<String, Integer> entry: jointNameToTrajectoryIndex.entrySet() )
         {        
            OneDoFJoint joint = currentRobotModel.getOneDoFJointByName( entry.getKey() );
            int index = entry.getValue();

            joint.setQ(  jointsWaypoint.position[index] );
            joint.setQd( jointsWaypoint.velocity[index] );        
         }
         currentRobotModel.updateFrames();

         // align the foot as usually
         ReferenceFrame pelvis    = currentRobotModel.getRootJoint().getFrameAfterJoint();
         ReferenceFrame movedFoot = currentRobotModel.getSoleFrame(RobotSide.RIGHT);

         RigidBodyTransform footToPelvis  = pelvis.getTransformToDesiredFrame( movedFoot );
         RigidBodyTransform worldToPelvis = new RigidBodyTransform();
         worldToPelvis.multiply( worldToFoot, footToPelvis );

         currentRobotModel.getRootJoint().setPositionAndRotation(worldToPelvis);

         currentRobotModel.updateFrames();

         TwistCalculator twistCalculator = new TwistCalculator(worldFrame,  currentRobotModel.getElevator() );
         twistCalculator.compute();

         //-----  store pelvis data --------
         worldToPelvis.get( packet.pelvisWorldOrientation[w], temp );
         packet.pelvisWorldPosition[w].set( temp );

         Twist twistToPack = new Twist();
         twistCalculator.packTwistOfBody(twistToPack, currentRobotModel.getElevator() );

         packet.pelvisLinearVelocity[w].set(  twistToPack.getLinearPartCopy()  );
         packet.pelvisAngularVelocity[w].set( twistToPack.getAngularPartCopy() );

         //-----  store chest data --------
         RigidBodyTransform worldToChest =  currentRobotModel.getChest().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame();
         worldToChest.get( packet.chestWorldOrientation[w] );

         twistCalculator.packTwistOfBody(twistToPack, currentRobotModel.getChest() );
         packet.chestAngularVelocity[w].set(  twistToPack.getAngularPartCopy() );
      }
      
      // check if parts of the trajectory packet can be set to null to decrease packet size:
      double epsilon = 1e-5;
      Vector3d zeroVector = new Vector3d();
      
      RigidBodyTransform pelvisToWorld = fullRobotModel.getRootJoint().getFrameAfterJoint().getTransformToWorldFrame();
      Point3d pelvisPosition = new Point3d();
      Quat4d pelvisOrientation = new Quat4d();
      pelvisToWorld.get(pelvisOrientation, pelvisPosition);
      
      RigidBodyTransform chestToWorld =  fullRobotModel.getChest().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame();
      Point3d chestPosition = new Point3d();
      Quat4d chestOrientation = new Quat4d();
      chestToWorld.get(chestOrientation, chestPosition);
      
      
      boolean pelvisLinearVelocityContent = false;
      boolean pelvisWorldPositionContent = false;
      boolean pelvisAngularVelocityContent = false;
      boolean pelvisWorldOrientationContent = false;
      
      boolean chestWorldOrientationContent = false;
      boolean chestAngulatVelocityContent = false;
      
      for (int n = 0; n < numWaypoints; n++)
      {
      
         
         if(!packet.pelvisLinearVelocity[n].epsilonEquals(zeroVector, epsilon))
         {
            pelvisLinearVelocityContent = true;
         }
         if(!packet.pelvisWorldPosition[n].epsilonEquals(pelvisPosition, epsilon))
         {
            pelvisWorldPositionContent = true;
         }
         if(!packet.pelvisWorldOrientation[n].epsilonEquals(pelvisOrientation, epsilon))
         {
            pelvisWorldOrientationContent = true;
         }
         if(!packet.pelvisAngularVelocity[n].epsilonEquals(zeroVector, epsilon))
         {
            pelvisAngularVelocityContent = true;
         }
         
         if(!packet.chestWorldOrientation[n].epsilonEquals(chestOrientation, epsilon))
         {
            chestWorldOrientationContent = true;
         }
         if(!packet.chestAngularVelocity[n].epsilonEquals(zeroVector, epsilon))
         {
            chestAngulatVelocityContent = true;
         }
      }
      
      if (!pelvisLinearVelocityContent)
      {
//         System.out.println("no pelvis linear velocity");
         packet.pelvisLinearVelocity = null;
      }
      if(!pelvisWorldPositionContent)
      {
//         System.out.println("no pelvis position content");
         packet.pelvisWorldPosition = null;
      }
      if(!pelvisWorldOrientationContent)
      {
//         System.out.println("no pelvis orientation content");
         packet.pelvisWorldOrientation = null;
      }
      if(!pelvisAngularVelocityContent)
      {
//         System.out.println("no pelvis angular velocity content");
         packet.pelvisAngularVelocity = null;
      }
      if(!chestWorldOrientationContent)
      {
//         System.out.println("no chest orientation content");
         packet.chestWorldOrientation = null;
      }
      if(!chestAngulatVelocityContent)
      {
//         System.out.println("no chest angular velocity content");
         packet.chestAngularVelocity = null;
      }
      
      return packet;
   }
}
