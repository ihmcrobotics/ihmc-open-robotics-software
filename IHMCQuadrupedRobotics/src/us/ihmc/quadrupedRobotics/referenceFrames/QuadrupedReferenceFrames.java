package us.ihmc.quadrupedRobotics.referenceFrames;

import java.util.EnumMap;

import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.RobotSpecificJointNames;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPhysicalProperties;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.containers.ContainerTools;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class QuadrupedReferenceFrames extends CommonQuadrupedReferenceFrames
{
   private final FullRobotModel fullRobotModel;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame bodyFrame, rootJointFrame;
   private final ZUpFrame bodyZUpFrame;

   private final EnumMap<NeckJointName, ReferenceFrame> neckReferenceFrames = ContainerTools.createEnumMap(NeckJointName.class);
   private final QuadrantDependentList<EnumMap<LegJointName, ReferenceFrame>> framesBeforeLegJoint = QuadrantDependentList.createListOfEnumMaps(LegJointName.class);
   private final QuadrantDependentList<EnumMap<LegJointName, ReferenceFrame>> framesAfterLegJoint = QuadrantDependentList.createListOfEnumMaps(LegJointName.class);
   private final QuadrantDependentList<ReferenceFrame> soleFrames = new QuadrantDependentList<ReferenceFrame>();
   
   private final QuadrantDependentList<ReferenceFrame> legAttachementFrames = new QuadrantDependentList<ReferenceFrame>();
   private final QuadrantDependentList<FramePoint> legAttachementPoints= new QuadrantDependentList<FramePoint>();

   private final SideDependentList<ReferenceFrame> sideDependentMidTrotLineZUpFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> sideDependentMidFeetZUpFrames = new SideDependentList<ReferenceFrame>();
   private final EndDependentList<ReferenceFrame> endDependentMidFeetZUpFrames = new EndDependentList<ReferenceFrame>();
   
   private final ReferenceFrame centerOfMassFrame;
   private final PoseReferenceFrame centerOfMassFrameWithRotation;
   private final ZUpFrame centerOfMassZUpFrame;
   private final FramePose centerOfMassPose;
   private final PoseReferenceFrame centerOfFourHipsFrame;
   private final FramePose centerOfFourHipsFramePose;
   private final FramePoint centerOfFourHipsFramePoint = new FramePoint();
   

   public QuadrupedReferenceFrames(SDFFullRobotModel fullRobotModel, QuadrupedJointNameMap quadrupedJointNameMap, QuadrupedPhysicalProperties quadrupedPhysicalProperties)
   {
      this.fullRobotModel = fullRobotModel;

      rootJointFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      bodyFrame = fullRobotModel.getRootJoint().getSuccessor().getBodyFixedFrame();
      centerOfMassPose = new FramePose(bodyFrame);

      bodyZUpFrame = new ZUpFrame(worldFrame, bodyFrame, "bodyZUpFrame");
      RobotSpecificJointNames robotJointNames = fullRobotModel.getRobotSpecificJointNames();
      
      for (NeckJointName neckJointName : robotJointNames.getNeckJointNames())
      {
         this.neckReferenceFrames.put(neckJointName, fullRobotModel.getNeckJoint(neckJointName).getFrameAfterJoint());
      }

      LegJointName[] legJointNames = quadrupedJointNameMap.getLegJointNames();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {         
         for (LegJointName legJointName : legJointNames)
         {
            String jointName = quadrupedJointNameMap.getLegJointName(robotQuadrant, legJointName);
            OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
            ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
            ReferenceFrame frameAfterJoint = oneDoFJoint.getFrameAfterJoint();
            
            framesBeforeLegJoint.get(robotQuadrant).put(legJointName, frameBeforeJoint);
            framesAfterLegJoint.get(robotQuadrant).put(legJointName, frameAfterJoint);
         }
         
         ReferenceFrame frameAfterKnee = framesAfterLegJoint.get(robotQuadrant).get(LegJointName.KNEE);

         TranslationReferenceFrame soleFrame = new TranslationReferenceFrame(robotQuadrant.toString() + "SoleFrame", frameAfterKnee);
         soleFrame.updateTranslation(quadrupedPhysicalProperties.getOffsetFromKneeToFoot(robotQuadrant));
         soleFrame.update();
         
         soleFrames.put(robotQuadrant, soleFrame);
         
         FramePoint legAttachmentPoint = new FramePoint();
         legAttachementPoints.put(robotQuadrant, legAttachmentPoint);
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant hindSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide);
         RobotQuadrant frontSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide);
         RobotQuadrant frontSoleQuadrantOppositeSide = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide.getOppositeSide());
         
         MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame(robotSide.getCamelCaseNameForStartOfExpression() + "MidFeetZUpFrame", worldFrame, soleFrames.get(hindSoleQuadrant), soleFrames.get(frontSoleQuadrant));
         sideDependentMidFeetZUpFrames.put(robotSide, midFeetZUpFrame);
         
         MidFrameZUpFrame midTrotLineZUpFrame = new MidFrameZUpFrame("hind" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Front" + robotSide.getOppositeSide().getCamelCaseNameForMiddleOfExpression() + "MidTrotLineZUpFrame", worldFrame, soleFrames.get(hindSoleQuadrant), soleFrames.get(frontSoleQuadrantOppositeSide));
         sideDependentMidTrotLineZUpFrames.put(robotSide, midTrotLineZUpFrame);
      }
      
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         RobotQuadrant leftSoleQuadrant = RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT);
         RobotQuadrant rightSoleQuadrant = RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT);
         
         MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame(robotEnd.getCamelCaseNameForStartOfExpression() + "MidFeetZUpFrame", worldFrame, soleFrames.get(leftSoleQuadrant), soleFrames.get(rightSoleQuadrant));
         endDependentMidFeetZUpFrames.put(robotEnd, midFeetZUpFrame);
      }
             
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      { 
         ReferenceFrame frameBeforeHipRoll = framesBeforeLegJoint.get(robotQuadrant).get(LegJointName.HIP_ROLL);         
         ReferenceFrame frameBeforeHipPitch = framesBeforeLegJoint.get(robotQuadrant).get(LegJointName.HIP_PITCH);
         
         FramePoint xyOffsetFromRollToPitch = new FramePoint(frameBeforeHipPitch);
         xyOffsetFromRollToPitch.changeFrame(frameBeforeHipRoll);
         
         TranslationReferenceFrame legAttachmentFrame = new TranslationReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "LegAttachementFrame", frameBeforeHipRoll);

         xyOffsetFromRollToPitch.setZ(0.0);
         
         legAttachmentFrame.updateTranslation(xyOffsetFromRollToPitch);
         legAttachementFrames.put(robotQuadrant, legAttachmentFrame);
      }
      
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, fullRobotModel.getElevator());
      
      centerOfMassPose.setToZero(centerOfMassFrame);
      centerOfMassPose.changeFrame(bodyFrame);
      
      centerOfMassFrameWithRotation = new PoseReferenceFrame("centerOfMassFrameWithRotation", bodyFrame);
      centerOfMassFrameWithRotation.setPoseAndUpdate(centerOfMassPose);
      centerOfMassZUpFrame = new ZUpFrame(worldFrame, centerOfMassFrameWithRotation, "centerOfMassZUpFrame");
      
      centerOfFourHipsFramePose = new FramePose(bodyFrame);
      centerOfFourHipsFrame = new PoseReferenceFrame("centerOfFourHipsFrame", bodyFrame);
      updateHipsCentroid();
      
      initializeCommonValues();
   }

   private void updateHipsCentroid()
   {
      centerOfFourHipsFramePose.setToZero(bodyFrame);
      centerOfFourHipsFramePoint.setToZero(bodyFrame);
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint legAttachmentPoint = legAttachementPoints.get(quadrant);
         legAttachmentPoint.setToZero(legAttachementFrames.get(quadrant));
         legAttachmentPoint.changeFrame(bodyFrame);
         centerOfFourHipsFramePoint.add(legAttachmentPoint);
         
      }
      centerOfFourHipsFramePoint.scale(0.25);
      centerOfFourHipsFramePose.setPosition(centerOfFourHipsFramePoint);
      centerOfFourHipsFrame.setPoseAndUpdate(centerOfFourHipsFramePose);
   }

   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }
   
   @Override
   public ReferenceFrame getBodyZUpFrame()
   {
      return bodyZUpFrame;
   }

   @Override
   public ReferenceFrame getSideDependentMidFeetZUpFrame(RobotSide robotSide)
   {
      return sideDependentMidFeetZUpFrames.get(robotSide);
   }

   @Override
   public ReferenceFrame getMidTrotLineZUpFrame(RobotQuadrant quadrantAssocaitedWithTrotLine)
   {
      if(quadrantAssocaitedWithTrotLine.isQuadrantInHind())
      {
         return sideDependentMidTrotLineZUpFrames.get(quadrantAssocaitedWithTrotLine.getSide());
      }
      return sideDependentMidTrotLineZUpFrames.get(quadrantAssocaitedWithTrotLine.getOppositeSide());
   }
   
   @Override
   public ReferenceFrame getEndDependentMidFeetZUpFrame(RobotEnd robotEnd)
   {
      return endDependentMidFeetZUpFrames.get(robotEnd);
   }
   
   @Override
   public ReferenceFrame getRootJointFrame()
   {
      return rootJointFrame;
   }
   
   public ReferenceFrame getNeckFrame(NeckJointName neckJointName)
   {
      return neckReferenceFrames.get(neckJointName);
   }

   @Override
   public ReferenceFrame getFrameBeforeLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return framesBeforeLegJoint.get(robotQuadrant).get(legJointName);
   }
   
   public ReferenceFrame getLegJointFrame(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return framesAfterLegJoint.get(robotQuadrant).get(legJointName);
   }
   
   @Override
   public ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant)
   {
      return legAttachementFrames.get(robotQuadrant);
   }
   
   @Override
   public ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant)
   {
      return getLegJointFrame(robotQuadrant, LegJointName.HIP_ROLL);
   }

   @Override
   public ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant)
   {
      return getLegJointFrame(robotQuadrant, LegJointName.HIP_PITCH);
   }

   @Override
   public ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant)
   {
      return getLegJointFrame(robotQuadrant, LegJointName.KNEE);
   }
   
   @Override
   public ReferenceFrame getFootFrame(RobotQuadrant robotQuadrant)
   {
      return soleFrames.get(robotQuadrant);
   }
   
   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      //return centerOfMassFrame;
      return centerOfMassFrameWithRotation;
   }

   public ReferenceFrame getCenterOfMassZUpFrame()
   {
      return centerOfMassZUpFrame;
   }

   @Override
   public QuadrantDependentList<ReferenceFrame> getFootReferenceFrames()
   {
      return soleFrames;
   }
   
   public void updateFrames()
   {
      fullRobotModel.updateFrames();
      bodyZUpFrame.update();
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleFrames.get(robotQuadrant).update();
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         sideDependentMidFeetZUpFrames.get(robotSide).update();
         sideDependentMidTrotLineZUpFrames.get(robotSide).update();
      }
      
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         endDependentMidFeetZUpFrames.get(robotEnd).update();
      }

      centerOfMassFrame.update();
      
      centerOfMassPose.setToZero(centerOfMassFrame);
      centerOfMassPose.changeFrame(bodyFrame);
      centerOfMassPose.setOrientation(new Quat4d());
      
      centerOfMassFrameWithRotation.setPoseAndUpdate(centerOfMassPose);
      centerOfMassZUpFrame.update();
      
      updateHipsCentroid();
   }

   @Override
   public ReferenceFrame getCenterOfFourHipsFrame()
   {
      return centerOfFourHipsFrame;
   }
}
