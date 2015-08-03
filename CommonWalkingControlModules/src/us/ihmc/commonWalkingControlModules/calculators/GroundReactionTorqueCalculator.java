package us.ihmc.commonWalkingControlModules.calculators;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class GroundReactionTorqueCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GroundTau");

   private final SideDependentList<YoFramePoint> groundTauStart = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> groundTau = new SideDependentList<YoFrameVector>();
   private final CommonHumanoidReferenceFrames commonHumanoidReferenceFrames;
   
   private final SideDependentList<ArrayList<GroundContactPoint>> contactPointList;
   
   public GroundReactionTorqueCalculator(SideDependentList<ArrayList<GroundContactPoint>> contactPointList, CommonHumanoidReferenceFrames commonHumanoidReferenceFrames, YoVariableRegistry parentRegistry,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactPointList = new SideDependentList<ArrayList<GroundContactPoint>>(contactPointList);
      this.commonHumanoidReferenceFrames = commonHumanoidReferenceFrames;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePoint groundTauStartPoint = new YoFramePoint("groundTauStart" + robotSide.getCamelCaseNameForMiddleOfExpression(), "",
                                               ReferenceFrame.getWorldFrame(), registry);
         groundTauStart.put(robotSide, groundTauStartPoint);

         YoFrameVector groundTauVector = new YoFrameVector("groundTauVector" + robotSide.getCamelCaseNameForMiddleOfExpression(), "",
               ReferenceFrame.getWorldFrame(), registry);
         groundTau.put(robotSide, groundTauVector);
      }

      parentRegistry.addChild(registry);
         
      if (yoGraphicsListRegistry != null)
         createDynamicGrapicVectors(yoGraphicsListRegistry);
   }


   private void createDynamicGrapicVectors(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("Ground-Foot Torques");

      double scaleFactor = 0.01;
      AppearanceDefinition appearance = YoAppearance.Purple();    // BlackMetalMaterial();

      for (RobotSide robotSide : RobotSide.values)
      {
         YoGraphicVector dynamicGraphicVector = new YoGraphicVector("GroundTau" + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                                        groundTauStart.get(robotSide), groundTau.get(robotSide), scaleFactor, appearance);
         yoGraphicsList.add(dynamicGraphicVector);
      }


      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }


   public FrameVector getGroundReactionTorque(RobotSide robotSide)
   {
      ReferenceFrame ankleZupFrame =  commonHumanoidReferenceFrames.getAnkleZUpFrame(robotSide);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameVector ret = new FrameVector(ankleZupFrame, 0.0, 0.0, 0.0);

      FramePoint ankleOrigin = new FramePoint(ankleZupFrame, 0.0, 0.0, 0.0);
      
      //Move the start of the display vector just outside of the shank link
      FramePoint ankleOriginShifted = new FramePoint(ankleZupFrame, 0.0, robotSide.negateIfRightSide(0.07), 0.0);
      ankleOriginShifted.changeFrame(worldFrame);
      groundTauStart.get(robotSide).set(ankleOriginShifted);
      
//      for (ContactPointName contactPointName : ContactPointName.values())
      final ArrayList<GroundContactPoint> contactPoints = contactPointList.get(robotSide);
      for(int i = 0; i < contactPoints.size(); i++)
      {
         GroundContactPoint groundContactPoint = contactPoints.get(i);
         // Compute r X f for each contact point. Use the ankle origin to calculate total moments about it
         FramePoint contactPointLocation = new FramePoint(worldFrame, groundContactPoint.getX(), groundContactPoint.getY(),
                                              groundContactPoint.getZ());
         contactPointLocation.changeFrame(ankleZupFrame);

         FrameVector vectorToContactPoint = new FrameVector(contactPointLocation);
         vectorToContactPoint.sub(ankleOrigin);


         Vector3d force = new Vector3d();
         groundContactPoint.getForce(force);
         FrameVector groundReactionForce = new FrameVector(worldFrame, force);

         groundReactionForce.changeFrame(ankleZupFrame);

         FrameVector torque = new FrameVector(ankleZupFrame);
         torque.cross(vectorToContactPoint, groundReactionForce);

         ret.add(torque);
      }


      ret.changeFrame(worldFrame);
      
      //For now, only display the z component. The other components over power the Z and then the Viz is worthless
      groundTau.get(robotSide).setZ(ret.getZ());

      return ret;
   }
}

