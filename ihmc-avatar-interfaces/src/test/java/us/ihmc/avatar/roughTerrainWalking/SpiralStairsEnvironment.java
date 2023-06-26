package us.ihmc.avatar.roughTerrainWalking;


import java.util.function.Consumer;

import org.fest.swing.util.System;

import boofcv.gui.d3.Orientation3D;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class SpiralStairsEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final AppearanceDefinition steppingStoneColor = YoAppearance.DarkGray();
   private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Spiral Stairs");
   private Consumer<FootstepDataListMessage> corruptor;
   //Environment details
   private final double spiralRadius;
   private final int noOfSteps;
   private final double spiralPitch;

   //Step details
   private final double stepSizeX;
   private final double stepSizeY;
   private final double stepSizeZ;
   
   //Platform details
   private double platformSizeX = 1.5;
   private double platformSizeY = 1.5;
   private double platformSizeZ;

   private double platformCenterX = 0;
   private double platformCenterY = 0;
   private double platformCenterZ;

   ReferenceFrame baseBlockFrame;
   ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private double stairAngleOffset=1.57;

   public SpiralStairsEnvironment(double stepSizeX, double stepSizeY, double stepSizeZ, double spiralRadius, double spiralPitch, int noOfSteps)
   {
      this.stepSizeX = stepSizeX;
      this.stepSizeY = stepSizeY;
      this.stepSizeZ = stepSizeZ;
      
      this.platformSizeZ=stepSizeZ;
      platformCenterZ = -platformSizeZ / 2;

      this.spiralRadius = spiralRadius;
      this.noOfSteps = noOfSteps;
      this.spiralPitch = spiralPitch;

      generateStartingPlatform();
      GenerateStairs();
   }

   private void generateStartingPlatform()
   {
      Point3D centerOfPlatform = new Point3D(platformCenterX, platformCenterY, platformCenterZ);
      AxisAngle orinetationOfPlatform = new AxisAngle(0, 0, 0);
      Box3D startingPlatform = new Box3D(centerOfPlatform, orinetationOfPlatform, platformSizeX, platformSizeY, platformSizeZ);
      terrainObject.addRotatableBox(startingPlatform, steppingStoneColor);
      

   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      // TODO Auto-generated method stub
      return terrainObject;
   }

   public void addStepToEnvironment(double stairCenterX, double stairCenterY, double stairCenterZ, double yawInRadians)
   {
      Step step = new Step(stepSizeX,
                           stepSizeY,
                           stepSizeZ,
                           stairCenterX,
                           stairCenterY,
                           stairCenterZ,
                           yawInRadians);
      terrainObject.addTerrainObject(step.createStepAsTerrainObject());
   }

   public void GenerateStairs()
   {
      for (int i = 0; i <= noOfSteps; i++)
      {
         double currentStairCenterZ = i * stepSizeZ;
         double spiralAngle = calculateStairAngle(currentStairCenterZ);
         double currentStairCenterX = calculateStairCenterX(spiralAngle);
         double currentStairCenterY = calculateStairCenterY(spiralAngle);
         addStepToEnvironment(currentStairCenterX, currentStairCenterY, currentStairCenterZWithOffset(currentStairCenterZ), spiralAngle);
//         java.lang.System.out.println("i:" + i + ";x:" + currentStairCenterX + "; y:" + currentStairCenterY + "; z:" + currentStairCenterZ + "; theta:" + spiralAngle);
      }
   }

   private double currentStairCenterZWithOffset(double currentStairCenterZ)
   {
      return currentStairCenterZ+stepSizeZ/2;
   }

   private double calculateStairCenterX(double spiralAngle)
   {
      // TODO Auto-generated method stub
      double StairCenterX=spiralRadius * Math.cos(spiralAngle)+stairCenterOffsetX();
      return StairCenterX;
   }

   private double stairCenterOffsetX()
   {
      // TODO Auto-generated method stub
      return platformSizeX/2;
   }

   private double calculateStairCenterY(double spiralAngle)
   {
      // TODO Auto-generated method stub
      double StairCenterY= spiralRadius * Math.sin(spiralAngle)+stepCenterOffsetY();
      return StairCenterY;
   }

   private double stepCenterOffsetY()
   {
      // TODO Auto-generated method stub
      return spiralRadius;
   }

   private double calculateStairAngle(double currentHeight)
   {
      // TODO Auto-generated method stub
      double stairAngle=currentHeight/spiralPitch- stairAngleOffset;
      return stairAngle;
   }
   private double footStepOffsetFromStairCenterX(double stanceWidth,double yawInRadians)
   {
      return Math.sin(yawInRadians+stairAngleOffset)*stanceWidth;
   }
   
   private double footStepOffsetFromStairCenterY(double stanceWidth,double yawInRadians)
   {
      return Math.cos(yawInRadians+stairAngleOffset)*stanceWidth;
   }
   
   private double individualFootstepCalculatorX(RobotSide robotSide,double offsetFromCenterX, double stairCenterX)
   {
      return stairCenterX+0.5 * robotSide.negateIfLeftSide(offsetFromCenterX);
   }
   
   private FootstepDataListMessage createStairsFootsteps(boolean oneStairAtAtTime, double stanceWidth, double swingDuration, double transferDuration)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();


      if (oneStairAtAtTime)
      {
         for (int i = 0; i < noOfSteps + 1; i++)
         {
            double stairCenterLocationZ=i * stepSizeZ;
            double footStepLocationZ=stairCenterLocationZ+stepSizeZ;
            double spiralAngle = calculateStairAngle(stairCenterLocationZ);
            double stairCenterX = calculateStairCenterX(spiralAngle);
            double stairCenterY = calculateStairCenterY(spiralAngle);
            
            double offSetFromX=footStepOffsetFromStairCenterX(stanceWidth,spiralAngle);
            double offSetFromY=footStepOffsetFromStairCenterY(stanceWidth,spiralAngle);
            
            java.lang.System.out.println("i:" + i + ";x:" + stairCenterX + "; y:" + stairCenterY + "; z:" + currentStairCenterZWithOffset(stairCenterLocationZ) + "; theta:" + spiralAngle);
            for (RobotSide robotSide : RobotSide.values)
            {
               
               double footStepLocationX=individualFootstepCalculatorX(robotSide,offSetFromX,stairCenterX);
               double footStepLocationY=individualFootstepCalculatorY(robotSide,offSetFromY,stairCenterY);
           
               Point3D footStepLocation = new Point3D(footStepLocationX, footStepLocationY, footStepLocationZ);
               AxisAngle footStepOrientation = new AxisAngle(spiralAngle+stairAngleOffset,0,0);
               FootstepDataMessage footstepData=HumanoidMessageTools.createFootstepDataMessage(robotSide, footStepLocation, footStepOrientation);
               footsteps.getFootstepDataList().add().set(footstepData);
               java.lang.System.out.println(robotSide.toString()+": X: "+footStepLocationX+": Y: "+footStepLocationY+": Z: "+footStepLocationZ);
               
            }
         }
      }
      else
      {
         RobotSide robotSide = RobotSide.LEFT;
         for (int i = 0; i < noOfSteps + 1; i++)
         {
            
            double stairCenterLocationZ=i * stepSizeZ;
            double footStepLocationZ=stairCenterLocationZ+stepSizeZ;
            double spiralAngle = calculateStairAngle(stairCenterLocationZ);
            double stairCenterX = calculateStairCenterX(spiralAngle);
            double stairCenterY = calculateStairCenterY(spiralAngle);
            
            double offSetFromX=footStepOffsetFromStairCenterX(stanceWidth,spiralAngle);
            double offSetFromY=footStepOffsetFromStairCenterY(stanceWidth,spiralAngle);
            
            java.lang.System.out.println("i:" + i + ";x:" + stairCenterX + "; y:" + stairCenterY + "; z:" + currentStairCenterZWithOffset(stairCenterLocationZ) + "; theta:" + spiralAngle);

            double footStepLocationX=individualFootstepCalculatorX(robotSide,offSetFromX,stairCenterX);
            double footStepLocationY=individualFootstepCalculatorY(robotSide,offSetFromY,stairCenterY);
        
            Point3D footStepLocation = new Point3D(footStepLocationX, footStepLocationY, footStepLocationZ);
            AxisAngle footStepOrientation = new AxisAngle(spiralAngle+stairAngleOffset,0,0);
            FootstepDataMessage footstepData=HumanoidMessageTools.createFootstepDataMessage(robotSide, footStepLocation, footStepOrientation);
            footsteps.getFootstepDataList().add().set(footstepData);
            java.lang.System.out.println(robotSide.toString()+": X: "+footStepLocationX+": Y: "+footStepLocationY+": Z: "+footStepLocationZ);
            robotSide = robotSide.getOppositeSide();
            
         }
         
      }
      return footsteps;
   }
   
   private void setSwingTrajSimple(FramePoint3DReadOnly initialStep, FramePoint3DReadOnly finalStep, FootstepDataMessage footstepData, double swingDuration)
   {
      footstepData.setTrajectoryType(FootstepDataMessage.TRAJECTORY_TYPE_CUSTOM);
      Point3D[] waypoints = new Point3D[] {new Point3D(), new Point3D()};
      waypoints[0].set(initialStep);
//      waypoints[0].addX(-stepLength * 0.3 );
//      waypoints[0].setY(finalStep.getY());
//      waypoints[0].addZ(stepHeight + 0.08);
//      
//      waypoints[1].set(initialStep);
//      waypoints[1].setY(-initialStep.getY());
//      waypoints[1].addX(stepLength * 0.8);
//      waypoints[1].addZ(0.02);
//      MessageTools.copyData(waypoints, footstepData.getCustomPositionWaypoints());
      
      footstepData.setLiftoffDuration(0.05);
   }
   
   private double individualFootstepCalculatorY(RobotSide robotSide, double offSetFromY, double stairCenterY)
   {
      return stairCenterY+0.5 * robotSide.negateIfRightSide(offSetFromY);
   }

   public FootstepDataListMessage generateStairsFootsteps(boolean oneStairAtATime,
                                                          double swingDuration,
                                                          double transferDuration,
                                                          double actualFootLength, 
                                                          double stanceWidth)
   {
      FootstepDataListMessage footsteps = createStairsFootsteps(oneStairAtATime, stanceWidth, swingDuration, transferDuration);
      EndToEndTestTools.setStepDurations(footsteps, swingDuration, transferDuration);
      if (corruptor != null)
         corruptor.accept(footsteps);
      addFootsteps(footsteps.getFootstepDataList(), footsteps.getFootstepDataList());
      footsteps.setAreFootstepsAdjustable(false);
      footsteps.setOffsetFootstepsHeightWithExecutionError(false);
      footsteps.setOffsetFootstepsWithExecutionError(false);
      footsteps.setTrustHeightOfFootsteps(true);
      return footsteps;
   }
   private void addFootsteps(Object<FootstepDataMessage> source, Object<FootstepDataMessage> destination)
   {
//      for (int i = 0; i < source.size(); i++)
//      {
//         destination.add().set(source.get(i));
//      }
   }
}
