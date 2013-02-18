package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;

public class LookaheadFinalDesiredICPCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final EnumYoVariable<DesiredICPCalculatorMethod> desiredICPCalculatorMethod = new EnumYoVariable<DesiredICPCalculatorMethod>("desiredICPCalculatorMethod", registry, DesiredICPCalculatorMethod.class);
  
   private final DynamicGraphicPosition finalDesiredICP;
   private final DynamicGraphicCoordinateSystem nextStepPose, nextNextStepPose;
   
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public LookaheadFinalDesiredICPCalculator(YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {

      finalDesiredICP = new DynamicGraphicPosition("finalDesiredIPC", "", registry, 0.005, YoAppearance.Yellow(), GraphicType.CROSS);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("FinalDesiredICP", finalDesiredICP);
      dynamicGraphicObjectsListRegistry.registerArtifact("FinalDesiredICP", finalDesiredICP.createArtifact());
      
//      nextStepPolygon = new DynamicGraphicPolygon("nextStepPolygon", null, "nextStepPolygon", "", registry, 0.02, YoAppearance.Azure());
//      nextNextStepPolygon = new DynamicGraphicPolygon("nextNextStepPolygon", null, "nextNextStepPolygon", "", registry, 0.02, YoAppearance.Azure());
      
      nextStepPose = new DynamicGraphicCoordinateSystem("nextStepPose", "", registry, 0.2);
      nextNextStepPose = new DynamicGraphicCoordinateSystem("nextNextStepPose", "", registry, 0.2);
      
      desiredICPCalculatorMethod.set(DesiredICPCalculatorMethod.STANDARD);
      
      parentRegistry.addChild(registry);
   }
   
   public FramePoint2d getDoubleSupportFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      switch (desiredICPCalculatorMethod.getEnumValue())
      {
      case STANDARD:
      {
         return getDoubleSupportFinalDesiredICPForWalkingStandard(transferToAndNextFootstepsData);
      }
      case NEW:
      {
         return getDoubleSupportFinalDesiredICPForWalkingStandard(transferToAndNextFootstepsData);
      }
      default:
      {
         throw new RuntimeException("Should not get here!"); 
      }
      }
   }
   
   public FramePoint2d getDoubleSupportFinalDesiredICPForWalkingStandard(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      
      FramePose transferToFootstepAnklePose = transferToFootstep.getPoseCopy();
      ContactablePlaneBody transferToFootContactablePlaneBody = transferToAndNextFootstepsData.getTransferToFootContactablePlaneBody();
      FrameConvexPolygon2d transferToFootPolygonInSoleFrame = transferToAndNextFootstepsData.getTransferToFootPolygonInSoleFrame();
      RobotSide transferToSide = transferToAndNextFootstepsData.getTransferToSide();

      
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      Footstep nextNextFootstep = transferToAndNextFootstepsData.getNextNextFootstep();
            
     
//      nextStepPose.setPosition(position)
      
//      if (footPolygonInSoleFrame.getReferenceFrame() != referenceFrames.getSoleFrame(transferToSide))
//      {
//         throw new RuntimeException("not in sole frame");
//      }
            
      Transform3D footstepAnkleToWorldTransform = new Transform3D();
      getTransformFromPoseToWorld(footstepAnkleToWorldTransform, transferToFootstepAnklePose);
      
      Transform3D ankleToSoleTransform = FootstepUtils.getAnkleToSoleTransform(transferToFootContactablePlaneBody);
      ankleToSoleTransform.invert();
      
      FramePoint2d centroid2d = transferToFootPolygonInSoleFrame.getCentroidCopy();
      FramePoint centroid = centroid2d.toFramePoint();      
      centroid.changeFrameUsingTransform(null, ankleToSoleTransform);
      centroid.changeFrameUsingTransform(worldFrame, footstepAnkleToWorldTransform);
      
      FramePoint pointOffsetFromCentroid = new FramePoint(centroid);

      double extraX = 0.0;    // 0.02
      double extraY = transferToSide.negateIfLeftSide(0.04);
      FrameVector offset = new FrameVector(null, extraX, extraY, 0.0);
      offset.changeFrameUsingTransform(worldFrame, footstepAnkleToWorldTransform);

      pointOffsetFromCentroid.changeFrame(offset.getReferenceFrame());
      pointOffsetFromCentroid.add(offset);

      finalDesiredICP.setPosition(pointOffsetFromCentroid.changeFrameCopy(worldFrame));
      
      FramePoint2d ret = pointOffsetFromCentroid.toFramePoint2d();
      return ret;
   }

   
   private static void getTransformFromPoseToWorld(Transform3D poseToWorldTransformToPack, FramePose framePose)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      framePose.changeFrame(worldFrame);
      framePose.getTransformFromPoseToFrame(poseToWorldTransformToPack);
   }

   private static enum DesiredICPCalculatorMethod
   {
         STANDARD, NEW;
   }
}
