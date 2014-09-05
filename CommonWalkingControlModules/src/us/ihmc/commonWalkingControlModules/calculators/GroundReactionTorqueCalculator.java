package us.ihmc.commonWalkingControlModules.calculators;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class GroundReactionTorqueCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GroundTau");

   private final SideDependentList<YoFramePoint> groundTauStart = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> groundTau = new SideDependentList<YoFrameVector>();
   private final CommonWalkingReferenceFrames commonWalkingReferenceFrames;
   
   private final SideDependentList<ArrayList<GroundContactPoint>> contactPointList;
   
   public GroundReactionTorqueCalculator(SideDependentList<ArrayList<GroundContactPoint>> contactPointList, CommonWalkingReferenceFrames commonWalkingReferenceFrames, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.contactPointList = new SideDependentList<ArrayList<GroundContactPoint>>(contactPointList);
      this.commonWalkingReferenceFrames = commonWalkingReferenceFrames;

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
         
      if (dynamicGraphicObjectsListRegistry != null)
         createDynamicGrapicVectors(dynamicGraphicObjectsListRegistry);
   }


   private void createDynamicGrapicVectors(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      YoGraphicsList dynamicGraphicObjectsList = new YoGraphicsList("Ground-Foot Torques");

      double scaleFactor = 0.01;
      AppearanceDefinition appearance = YoAppearance.Purple();    // BlackMetalMaterial();

      for (RobotSide robotSide : RobotSide.values)
      {
         YoGraphicVector dynamicGraphicVector = new YoGraphicVector("GroundTau" + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                                        groundTauStart.get(robotSide), groundTau.get(robotSide), scaleFactor, appearance);
         dynamicGraphicObjectsList.add(dynamicGraphicVector);
      }


      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
   }


   public FrameVector getGroundReactionTorque(RobotSide robotSide)
   {
      ReferenceFrame ankleZupFrame =  commonWalkingReferenceFrames.getAnkleZUpFrame(robotSide);
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

