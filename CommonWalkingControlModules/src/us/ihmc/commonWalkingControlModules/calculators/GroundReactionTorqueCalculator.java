package us.ihmc.commonWalkingControlModules.calculators;

import java.util.ArrayList;

import javax.media.j3d.Appearance;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

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
      this.contactPointList = contactPointList;
      this.commonWalkingReferenceFrames = commonWalkingReferenceFrames;

      for (RobotSide robotSide : RobotSide.values())
      {
         YoFramePoint groundTauStartPoint = new YoFramePoint("groundTauStart" + robotSide.getCamelCaseNameForMiddleOfExpression(), "",
                                               ReferenceFrame.getWorldFrame(), registry);
         groundTauStart.put(robotSide, groundTauStartPoint);

         YoFrameVector groundTauVector = new YoFrameVector("groundTauVector" + robotSide.getCamelCaseNameForMiddleOfExpression(), "",
               ReferenceFrame.getWorldFrame(), registry);
         groundTau.put(robotSide, groundTauVector);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
         
      if (dynamicGraphicObjectsListRegistry != null)
         createDynamicGrapicVectors(dynamicGraphicObjectsListRegistry);
   }


   private void createDynamicGrapicVectors(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("Ground-Foot Torques");

      double scaleFactor = 0.01;
      Appearance appearance = YoAppearance.Purple();    // BlackMetalMaterial();

      for (RobotSide robotSide : RobotSide.values())
      {
         DynamicGraphicVector dynamicGraphicVector = new DynamicGraphicVector("GroundTau" + robotSide.getCamelCaseNameForMiddleOfExpression(),
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
      groundTauStart.get(robotSide).set(ankleOriginShifted.changeFrameCopy(worldFrame));
      
//      for (ContactPointName contactPointName : ContactPointName.values())
      for(GroundContactPoint groundContactPoint : contactPointList.get(robotSide))
      {
         // Compute r X f for each contact point. Use the ankle origin to calculate total moments about it
         FramePoint contactPointLocation = new FramePoint(worldFrame, groundContactPoint.x.getDoubleValue(), groundContactPoint.y.getDoubleValue(),
                                              groundContactPoint.z.getDoubleValue());
         contactPointLocation = contactPointLocation.changeFrameCopy(ankleZupFrame);

         FrameVector vectorToContactPoint = new FrameVector(contactPointLocation);
         vectorToContactPoint.sub(ankleOrigin);


         FrameVector groundReactionForce = new FrameVector(worldFrame, groundContactPoint.fx.getDoubleValue(), groundContactPoint.fy.getDoubleValue(),
                                              groundContactPoint.fz.getDoubleValue());

         groundReactionForce = groundReactionForce.changeFrameCopy(ankleZupFrame);

         FrameVector torque = new FrameVector(ankleZupFrame);
         torque.cross(vectorToContactPoint, groundReactionForce);

         ret.add(torque);
      }


      ret = ret.changeFrameCopy(worldFrame);
      
      //For now, only display the z component. The other components over power the Z and then the Viz is worthless
      groundTau.get(robotSide).setZ(ret.getZ());

      return ret;
   }
}

