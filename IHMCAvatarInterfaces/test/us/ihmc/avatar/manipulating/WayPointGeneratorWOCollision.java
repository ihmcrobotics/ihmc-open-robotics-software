package us.ihmc.avatar.manipulating;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.tools.io.printing.PrintTools;

public class WayPointGeneratorWOCollision implements ConstrainedWayPointGenerator
{
   CollisionConstraintCondition constraint;
      
   public Pose initialPoint = new Pose();
   public Pose finalPoint = new Pose();

   public int numberOfWayPoints = 0;
   public double totalTrajectoryTime = 0.0;

//   public Point3d[] positionOfWayPoints = null;
//   public Quat4d[] orientationOfWayPoints = null;
      
   public Pose[] wayPoints;

   private static final double EPS = 0.000001;
   private double referenceForDistance = 0;

   public WayPointGeneratorWOCollision(CollisionConstraintCondition collisionState)
   {
      this.constraint = collisionState;       
   }
   
   @Override
   public void setInitialPose(Pose pose)
   {
      initialPoint = pose;      
   }

   @Override
   public void setFinalPose(Pose pose)
   {
      finalPoint = pose;      
   }

   @Override
   public Pose getWayPoint(int indexOfWayPoint)
   {      
      return wayPoints[indexOfWayPoint];
   }

   @Override
   public void setNumberOfWayPoints(int numPoints)
   {
      numberOfWayPoints = numPoints;
      wayPoints = new Pose[numberOfWayPoints];
      for (int i = 0; i < numberOfWayPoints; i++)
      {
         wayPoints[i] = new Pose();
      }      
   }

   @Override
   public void initializeWayPoints()
   {
      referenceForDistance = initialPoint.getPoint().distance(finalPoint.getPoint())/(numberOfWayPoints-1);
      // position - linear
      for (int i = 0; i < numberOfWayPoints; i++)
      {
         wayPoints[i].getPoint().setX(initialPoint.getX() + (finalPoint.getX() - initialPoint.getX()) / (numberOfWayPoints - 1) * (i));
         wayPoints[i].getPoint().setY(initialPoint.getY() + (finalPoint.getY() - initialPoint.getY()) / (numberOfWayPoints - 1) * (i));
         wayPoints[i].getPoint().setZ(initialPoint.getZ() + (finalPoint.getZ() - initialPoint.getZ()) / (numberOfWayPoints - 1) * (i));
      }

      // orientation - linearize the angle of the rotational vector from initorientation to goalorientation
      Quat4d toGoalOrientation = new Quat4d();
      Quat4d toWayPointOrientation = new Quat4d();
      Quat4d inverseInitialOrientation = new Quat4d(initialPoint.getOrientation());

      inverseInitialOrientation.inverse();

      toGoalOrientation.mul(inverseInitialOrientation, finalPoint.getOrientation());

      AxisAngle4d toGoal = new AxisAngle4d();
      Vector3d toGoalVector = new Vector3d();
            
      RotationTools.convertQuaternionToRotationVector(toGoalOrientation, toGoalVector);
      RotationTools.convertRotationVectorToAxisAngle(toGoalVector, toGoal);
      
      AxisAngle4d toWayPoint = new AxisAngle4d(toGoal);
      double fullAngle = toWayPoint.angle;

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         toWayPoint.angle = fullAngle * (i) / (numberOfWayPoints - 1);
         
         Vector3d tempRV = new Vector3d(toWayPoint.x*toWayPoint.angle, toWayPoint.y*toWayPoint.angle, toWayPoint.z*toWayPoint.angle);    
         RotationTools.convertRotationVectorToQuaternion(tempRV, toWayPointOrientation);
         wayPoints[i].getOrientation().mul(initialPoint.getOrientation(), toWayPointOrientation);
      }
      
      for (int i=0;i<numberOfWayPoints;i++)
      {
         //wayPoints[i].setPose(positionOfWayPoints[i], orientationOfWayPoints[i]);
         PrintTools.info("Initialized Way Point ["+i+"] is "+ wayPoints[i].getPoint().x+" "+ wayPoints[i].getPoint().y+" "+ wayPoints[i].getPoint().z);
      }
   }

   @Override
   public void setWayPoint(int indexOfWayPoint, Pose pose)
   {
      wayPoints[indexOfWayPoint].setPosition(pose.getPoint());      
      wayPoints[indexOfWayPoint].setOrientation(pose.getOrientation());      
   }
   
   public void setTotalTrajectoryTime(double totalTime)
   {
      totalTrajectoryTime = totalTime;
   }

   public int getNumberOfWayPoints()
   {
      return this.numberOfWayPoints;
   }

   public double getTotalTrajectoryTime()
   {
      return this.totalTrajectoryTime;
   }

   @Override
   public Point3d getWayPointPosition(int indexOfWayPoint) // indexOfWayPoint is .. 0:numberOfWayPoints-1
   {
      return wayPoints[indexOfWayPoint].getPoint();
   }

   @Override
   public Quat4d getWayPointOrientation(int indexOfWayPoint)
   {
      return wayPoints[indexOfWayPoint].getOrientation();
   }

   @Override
   public ArrayList<Graphics3DObject> createXYZAxisOfWayPoint(int indexOfWayPoint)
   {
      double axisHeight = 0.06;
      double axisRadius = 0.005;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      retX.translate(getWayPointPosition(indexOfWayPoint));
      retY.translate(getWayPointPosition(indexOfWayPoint));
      retZ.translate(getWayPointPosition(indexOfWayPoint));

      Quat4d qtWayPoint = new Quat4d();
      Quat4d qtAlpha = new Quat4d();
      Quat4d qtAxis = new Quat4d();
      AxisAngle4d rvAxis = new AxisAngle4d();
      Vector3d rvTemp = new Vector3d();
      qtWayPoint = getWayPointOrientation(indexOfWayPoint);

      RotationTools.convertQuaternionToRotationVector(qtWayPoint, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retZ.rotate(rvAxis);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      
      RotationTools.convertRotationVectorToQuaternion(new Vector3d(0, Math.PI / 2, 0), qtAlpha);
      qtAxis.mul(qtWayPoint, qtAlpha);

      RotationTools.convertQuaternionToRotationVector(qtAxis, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      
      retX.rotate(rvAxis);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      RotationTools.convertRotationVectorToQuaternion(new Vector3d(-Math.PI / 2, 0, 0), qtAlpha);
      qtAxis.mul(qtWayPoint, qtAlpha);

      RotationTools.convertQuaternionToRotationVector(qtAxis, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retY.rotate(rvAxis);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }   

   @Override
   public void generateWayPoints()
   {     
      int indexOfInvalidWayPoint;
      
      
      for (int i = 1; i < numberOfWayPoints; i++)
      {
         boolean resultOfConstraint = getResultForPoint(i);
         PrintTools.info("generate "+i+"th waypoint " + resultOfConstraint);
         
         if (resultOfConstraint == false)
         {
            PrintTools.info("Finding New waypoint ["+i+"]");
            
            indexOfInvalidWayPoint = i;
            setWayPoint(indexOfInvalidWayPoint, getBestNextPose(getWayPoint(indexOfInvalidWayPoint-1), MethodOptimizeNewWayPoint.QPforDisplacement, 200));
            reGenerateWayPoints(indexOfInvalidWayPoint);            
         }
      }

   }

   @Override
   public boolean getResultForPoint(Point3d wayPointPosition, Quat4d wayPointOrientation)
   {
      constraint.ghostRobot.updateJointOfGhost(wayPointPosition, wayPointOrientation);
      constraint.updateCurrentState();
      
      return constraint.getResult();
   }

   @Override
   public boolean getResultForPoint(int indexOfWayPoint)
   {
      return getResultForPoint(wayPoints[indexOfWayPoint].getPoint(), wayPoints[indexOfWayPoint].getOrientation());
   }
   
   @Override
   public boolean getResultForPoint(Pose pose)
   {
      return getResultForPoint(pose.getPoint(), pose.getOrientation());
   }


   @Override
   public void reGenerateWayPoints(int startIndexOfWayPoint)
   {
      double totalDistance = wayPoints[startIndexOfWayPoint].getPoint().distance(finalPoint.getPoint());
      int numberOfWayPointsToFinal = (int) Math.round(totalDistance/referenceForDistance) + 1;
      int newNumberOfWayPoints = startIndexOfWayPoint + numberOfWayPointsToFinal;
      numberOfWayPoints = newNumberOfWayPoints;
      
      PrintTools.info("Total distance is "+totalDistance +" reference Distance is "+referenceForDistance+" result is "+newNumberOfWayPoints);
      
      Pose newWayPoints[] = new Pose[numberOfWayPoints];
      
      for (int i = 0;i < numberOfWayPoints; i++)
      {
         newWayPoints[i] = new Pose();
      }
      for (int i =0;i<=startIndexOfWayPoint;i++)
      {
         newWayPoints[i] = new Pose(wayPoints[i]);
      }
      
      // position - linear
      for (int i = startIndexOfWayPoint; i < numberOfWayPoints; i++)
      {
         newWayPoints[i].getPoint().setX(getWayPointPosition(startIndexOfWayPoint).x + (finalPoint.getX() - getWayPointPosition(startIndexOfWayPoint).x)
               / (numberOfWayPoints - startIndexOfWayPoint - 1) * (i - startIndexOfWayPoint));
         newWayPoints[i].getPoint().setY(getWayPointPosition(startIndexOfWayPoint).y + (finalPoint.getY() - getWayPointPosition(startIndexOfWayPoint).y)
               / (numberOfWayPoints - startIndexOfWayPoint - 1) * (i - startIndexOfWayPoint));
         newWayPoints[i].getPoint().setZ(getWayPointPosition(startIndexOfWayPoint).z + (finalPoint.getZ() - getWayPointPosition(startIndexOfWayPoint).z)
               / (numberOfWayPoints - startIndexOfWayPoint - 1) * (i - startIndexOfWayPoint));
      }    
      
      // orientation - linearize the angle of the rotational vector
      Quat4d toGoalOrientation = new Quat4d();
      Quat4d toWayPointOrientation = new Quat4d();
      Quat4d inverseInitialOrientation = new Quat4d(getWayPointOrientation(startIndexOfWayPoint));

      inverseInitialOrientation.inverse();
      toGoalOrientation.mul(inverseInitialOrientation, finalPoint.getOrientation());

      AxisAngle4d toGoal = new AxisAngle4d();
      Vector3d toGoalRV = new Vector3d();
      RotationTools.convertQuaternionToRotationVector(toGoalOrientation, toGoalRV);
      RotationTools.convertRotationVectorToAxisAngle(toGoalRV, toGoal);
      AxisAngle4d toWayPoint = new AxisAngle4d(toGoal);
      double fullAngle = toWayPoint.angle;
      for (int i = startIndexOfWayPoint; i < numberOfWayPoints; i++)
      {
         toWayPoint.angle = fullAngle * (i - startIndexOfWayPoint) / (numberOfWayPoints - startIndexOfWayPoint - 1);
         Vector3d toWayPointRV = new Vector3d(toWayPoint.x*toWayPoint.angle, toWayPoint.y*toWayPoint.angle, toWayPoint.z*toWayPoint.angle);
         RotationTools.convertRotationVectorToQuaternion(toWayPointRV, toWayPointOrientation);
         newWayPoints[i].getOrientation().mul(getWayPointOrientation(startIndexOfWayPoint), toWayPointOrientation);
      }
      
      setNumberOfWayPoints(numberOfWayPoints);
      this.wayPoints = newWayPoints;
      
      
      
      
   }

 


   private Pose getRandomWayPoint(Point3d centerOfFrame, double maxRadius)
   {
      Pose ret = new Pose();
      
      double maximumRadius = maxRadius;
      
      double radiusOfRandom;
      double thetaOfRandom;
      double phiOfRandom;
      double uOfRandom, vOfRandom, wOfRandom;
      Random randomManager = new Random();
      
      radiusOfRandom = randomManager.nextDouble()*maximumRadius;
      radiusOfRandom = maximumRadius;
      thetaOfRandom = (randomManager.nextDouble()-0.5)*2*Math.PI;
      phiOfRandom = Math.asin(2*(randomManager.nextDouble()-0.5));
      
      uOfRandom = (randomManager.nextDouble()-0.5)*2*Math.PI;
      vOfRandom = (randomManager.nextDouble()-0.5)*2*Math.PI;
      wOfRandom = (randomManager.nextDouble()-0.5)*2*Math.PI;
      
      Point3d randomTranslation = new Point3d(radiusOfRandom*Math.cos(phiOfRandom)*Math.cos(thetaOfRandom)
                                              , radiusOfRandom*Math.cos(phiOfRandom)*Math.sin(thetaOfRandom)
                                              , radiusOfRandom*Math.sin(phiOfRandom));
      
      Quat4d randomRotation = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(wOfRandom, vOfRandom, uOfRandom, randomRotation);
      
      ret.setPosition(new Point3d(randomTranslation.x+centerOfFrame.x, randomTranslation.y+centerOfFrame.y, randomTranslation.z+centerOfFrame.z));
      ret.setOrientation(randomRotation);
      
      return ret;
   }
   
   public ArrayList<Graphics3DObject> createRandomXYZAxis()
   {
      double axisHeight = 0.0075;
      double axisRadius = 0.0025;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      Pose randomWayPoint = getRandomWayPoint(new Point3d(1.0, 0.0, 2.0), 0.1);      
            
      Point3d centerPoint = new Point3d();
      randomWayPoint.getPosition(centerPoint);

      retX.translate(centerPoint);
      retY.translate(centerPoint);
      retZ.translate(centerPoint);

      Quat4d qtCenter = new Quat4d(randomWayPoint.getOrientation());
      Quat4d qtAlpha = new Quat4d();
      Quat4d qtAxis = new Quat4d();
      AxisAngle4d rvAxis = new AxisAngle4d();

      Vector3d rvTemp = new Vector3d();
      RotationTools.convertQuaternionToRotationVector(qtCenter, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retZ.rotate(rvAxis);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      RotationTools.convertRotationVectorToQuaternion(new Vector3d(0, Math.PI / 2, 0), qtAlpha);
      qtAxis.mul(qtCenter, qtAlpha);

      RotationTools.convertQuaternionToRotationVector(qtAxis, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retX.rotate(rvAxis);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      RotationTools.convertRotationVectorToQuaternion(new Vector3d(-Math.PI / 2, 0, 0), qtAlpha);
      qtAxis.mul(qtCenter, qtAlpha);

      RotationTools.convertQuaternionToRotationVector(qtAxis, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retY.rotate(rvAxis);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }
   
   private double getScore(Pose poseOne, Pose poseTwo)
   {      
      double distanceScore;
      double angleScore;
      
      distanceScore = poseOne.getPoint().distance(poseTwo.getPoint());
                 
      Quat4d toGoal = new Quat4d();
      Quat4d invOfInitial = new Quat4d(poseOne.getOrientation());
      invOfInitial.inverse();
      toGoal.mul(invOfInitial, poseTwo.getOrientation());
      
      AxisAngle4d toGoalAxis = new AxisAngle4d();
      Vector3d toGoalRV = new Vector3d();
      
      RotationTools.convertQuaternionToRotationVector(toGoal, toGoalRV);
      RotationTools.convertRotationVectorToAxisAngle(toGoalRV, toGoalAxis);
      
      angleScore = toGoalAxis.angle;
      
      return (1.0*distanceScore + 1.0*angleScore);
   }
   
   private enum MethodOptimizeNewWayPoint
   {
      QPforDisplacement,
      QPforJointDisplacement
   }
   
   private Pose getBestNextPose(Pose originPose, MethodOptimizeNewWayPoint method, int numberOfTrial)
   {
     Pose bestPose = new Pose();
     Pose curPose;
     double bestScore = Double.MAX_VALUE;
     double curScore = 0;     
          
     for (int i=0;i<numberOfTrial;i++)
     {
        curPose = getRandomWayPoint(originPose.getPoint(), referenceForDistance);
        if(getResultForPoint(curPose) == true)
        {
           switch(method)
           {
           case QPforDisplacement:
              curScore = getScore(curPose, finalPoint);
              break;
           case QPforJointDisplacement:
              
              break;
           }
        }
        else
           curScore = Double.MAX_VALUE;
        
        
        if(curScore < bestScore)
        {
           bestPose = curPose;
           bestScore = curScore;
        }
        
     }
     
     PrintTools.info("Best Pose is "+bestPose.getPoint().x+" "+bestPose.getPoint().y+" "+bestPose.getPoint().z);
     
     return bestPose;
   }
   
   
   
   
  


  
}
