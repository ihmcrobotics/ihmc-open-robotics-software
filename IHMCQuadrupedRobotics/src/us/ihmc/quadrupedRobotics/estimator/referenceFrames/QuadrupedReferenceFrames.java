package us.ihmc.quadrupedRobotics.estimator.referenceFrames;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class QuadrupedReferenceFrames extends CommonQuadrupedReferenceFrames
{
   private final FullRobotModel fullRobotModel;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Quaternion IDENTITY_QUATERNION = new Quaternion();

   private final ReferenceFrame bodyFrame, rootJointFrame;
   private final ZUpFrame bodyZUpFrame;

   private final EnumMap<QuadrupedJointName, ReferenceFrame> namedReferenceFrames = new EnumMap<>(QuadrupedJointName.class);
   private final Map<QuadrupedJointName, ReferenceFrame> framesBeforeLegJoint = new EnumMap<>(QuadrupedJointName.class);
   private final Map<QuadrupedJointName, ReferenceFrame> framesAfterLegJoint = new EnumMap<>(QuadrupedJointName.class);
   private final QuadrantDependentList<ReferenceFrame> soleFrames = new QuadrantDependentList<ReferenceFrame>();

   private final QuadrantDependentList<PoseReferenceFrame> tripleSupportFrames = new QuadrantDependentList<PoseReferenceFrame>();
   private final QuadrantDependentList<FramePose> tripleSupportCentroidPoses = new QuadrantDependentList<FramePose>();

   private final FramePose supportPolygonCentroidWithNominalRotation = new FramePose(ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame supportPolygonCentroidFrameWithNominalRotation;
   private final ZUpFrame supportPolygonCentroidZUpFrame;

   private final QuadrantDependentList<ReferenceFrame> legAttachementFrames = new QuadrantDependentList<ReferenceFrame>();
   private final QuadrantDependentList<FramePoint> legAttachementPoints= new QuadrantDependentList<FramePoint>();

   private final SideDependentList<ReferenceFrame> sideDependentMidFeetZUpFrames = new SideDependentList<ReferenceFrame>();

   private final ReferenceFrame centerOfMassFrame;
   private final PoseReferenceFrame centerOfMassFrameWithRotation;
   private final ZUpFrame centerOfMassZUpFrame;
   private final FramePose centerOfMassPose;
   private final PoseReferenceFrame centerOfFourHipsFrame;
   private final FramePose centerOfFourHipsFramePose;
   private final FramePoint centerOfFourHipsFramePoint = new FramePoint();

   private final PoseReferenceFrame centerOfFourFeetFrameWithBodyRotation;
   private final FramePose centerOfFourFeetFramePose;
   private final FramePoint centerOfFourFeetFramePoint = new FramePoint();

   private QuadrupedSupportPolygon supportPolygonForCentroids = new QuadrupedSupportPolygon();

   public QuadrupedReferenceFrames(FullQuadrupedRobotModel fullRobotModel, QuadrupedPhysicalProperties quadrupedPhysicalProperties)
   {
      this.fullRobotModel = fullRobotModel;

      rootJointFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      bodyFrame = fullRobotModel.getRootJoint().getSuccessor().getBodyFixedFrame();
      centerOfMassPose = new FramePose(bodyFrame);

      bodyZUpFrame = new ZUpFrame(worldFrame, bodyFrame, "bodyZUpFrame");

      for (QuadrupedJointName jointName : QuadrupedJointName.values)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
         if (joint != null)
         {
            ReferenceFrame frame = joint.getFrameAfterJoint();
            namedReferenceFrames.put(jointName, frame);
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (OneDoFJoint oneDoFJoint : fullRobotModel.getLegOneDoFJoints(robotQuadrant))
         {
            ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
            ReferenceFrame frameAfterJoint = oneDoFJoint.getFrameAfterJoint();

            QuadrupedJointName legJointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
            framesBeforeLegJoint.put(legJointName, frameBeforeJoint);
            framesAfterLegJoint.put(legJointName, frameAfterJoint);
         }

         QuadrupedJointName kneeJointName = QuadrupedJointName.getName(robotQuadrant, LegJointName.KNEE_PITCH);
         ReferenceFrame frameAfterKnee = framesAfterLegJoint.get(kneeJointName);

         TranslationReferenceFrame soleFrame = new TranslationReferenceFrame(robotQuadrant.toString() + "SoleFrame", frameAfterKnee);
         soleFrame.updateTranslation(quadrupedPhysicalProperties.getOffsetFromJointBeforeFootToSole(robotQuadrant));
         soleFrame.update();

         soleFrames.set(robotQuadrant, soleFrame);

         FramePoint legAttachmentPoint = new FramePoint();
         legAttachementPoints.set(robotQuadrant, legAttachmentPoint);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant hindSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide);
         RobotQuadrant frontSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide);

         MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame(robotSide.getCamelCaseNameForStartOfExpression() + "MidFeetZUpFrame", worldFrame, soleFrames.get(hindSoleQuadrant), soleFrames.get(frontSoleQuadrant));
         sideDependentMidFeetZUpFrames.put(robotSide, midFeetZUpFrame);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedJointName hipRollJointName = QuadrupedJointName.getName(robotQuadrant, LegJointName.HIP_ROLL);
         ReferenceFrame frameBeforeHipRoll = framesBeforeLegJoint.get(hipRollJointName);

         QuadrupedJointName hipPitchJointName = QuadrupedJointName.getName(robotQuadrant, LegJointName.HIP_PITCH);
         ReferenceFrame frameBeforeHipPitch = framesBeforeLegJoint.get(hipPitchJointName);

         FramePoint xyOffsetFromRollToPitch = new FramePoint(frameBeforeHipPitch);
         xyOffsetFromRollToPitch.changeFrame(frameBeforeHipRoll);

         TranslationReferenceFrame legAttachmentFrame = new TranslationReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "LegAttachementFrame", frameBeforeHipRoll);

         xyOffsetFromRollToPitch.setZ(0.0);

         legAttachmentFrame.updateTranslation(xyOffsetFromRollToPitch);
         legAttachementFrames.set(robotQuadrant, legAttachmentFrame);


         FramePose tripleSupportCentroidPose = new FramePose(worldFrame);
         PoseReferenceFrame tripleSupport = new PoseReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "TripleSupportFrame", tripleSupportCentroidPose);

         tripleSupportCentroidPoses.set(robotQuadrant, tripleSupportCentroidPose);
         tripleSupportFrames.set(robotQuadrant, tripleSupport);
      }

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, fullRobotModel.getElevator());

      centerOfMassPose.setToZero(centerOfMassFrame);
      centerOfMassPose.changeFrame(bodyFrame);

      centerOfMassFrameWithRotation = new PoseReferenceFrame("centerOfMassFrameWithRotation", bodyFrame);
      centerOfMassFrameWithRotation.setPoseAndUpdate(centerOfMassPose);
      centerOfMassZUpFrame = new ZUpFrame(worldFrame, centerOfMassFrameWithRotation, "centerOfMassZUpFrame");

      centerOfFourHipsFramePose = new FramePose(bodyFrame);
      centerOfFourHipsFrame = new PoseReferenceFrame("centerOfFourHipsFrame", bodyFrame);

      centerOfFourFeetFramePose = new FramePose(bodyFrame);
      centerOfFourFeetFrameWithBodyRotation = new PoseReferenceFrame("centerOfFourFeetFrame", bodyFrame);
      supportPolygonCentroidFrameWithNominalRotation = new PoseReferenceFrame("centerOfFourFeetWithSupportPolygonRotation", supportPolygonCentroidWithNominalRotation);
      supportPolygonCentroidZUpFrame = new ZUpFrame(worldFrame, supportPolygonCentroidFrameWithNominalRotation, "centerFootPolygonZUp");

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
      }

      centerOfMassFrame.update();

      centerOfMassPose.setToZero(centerOfMassFrame);
      centerOfMassPose.changeFrame(bodyFrame);
      centerOfMassPose.setOrientation(IDENTITY_QUATERNION);

      centerOfMassFrameWithRotation.setPoseAndUpdate(centerOfMassPose);
      centerOfMassZUpFrame.update();

      updateCentroids();
   }

   private void updateCentroids()
   {
      updateHipsCentroid();
      updateCenterOfFeetUsingBodyForRotationPart();
      updateTripleSupportCentroids();
      updateCenterOfFeetUsingNominalsForRotationPart();
   }

   private void updateCenterOfFeetUsingBodyForRotationPart()
   {
      updateSupportPolygon(null,supportPolygonForCentroids);

      centerOfFourFeetFramePoint.changeFrame(worldFrame);
      supportPolygonForCentroids.getCentroid(centerOfFourFeetFramePoint);

      centerOfFourFeetFramePoint.changeFrame(bodyFrame);
      centerOfFourFeetFramePose.setToZero(bodyFrame);

      centerOfFourFeetFramePose.setPosition(centerOfFourFeetFramePoint);
      centerOfFourFeetFrameWithBodyRotation.setPoseAndUpdate(centerOfFourFeetFramePose);
   }

   private void updateTripleSupportCentroids()
   {
      for(RobotQuadrant swingLeg : RobotQuadrant.values)
      {
         updateSupportPolygon(swingLeg, supportPolygonForCentroids);
         FramePose framePose = tripleSupportCentroidPoses.get(swingLeg);
         supportPolygonForCentroids.getWeightedCentroidFramePoseAveragingLowestZHeightsAcrossEnds(framePose);

         PoseReferenceFrame tripleSupportFrame = tripleSupportFrames.get(swingLeg);
         tripleSupportFrame.setPoseAndUpdate(framePose);
      }
   }

   private void updateCenterOfFeetUsingNominalsForRotationPart()
   {
      updateSupportPolygon(null, supportPolygonForCentroids);
      supportPolygonForCentroids.getCentroidFramePoseAveragingLowestZHeightsAcrossEnds(supportPolygonCentroidWithNominalRotation);
      supportPolygonCentroidFrameWithNominalRotation.setPoseAndUpdate(supportPolygonCentroidWithNominalRotation);
      supportPolygonCentroidZUpFrame.update();
   }

   private final FramePoint soleFramePointTemp = new FramePoint();
   private void updateSupportPolygon(RobotQuadrant swingFoot, QuadrupedSupportPolygon supportPolygon)
   {
      supportPolygon.clear();
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if(robotQuadrant.equals(swingFoot))
         {
            continue;
         }
         ReferenceFrame soleFrame = soleFrames.get(robotQuadrant);
         soleFramePointTemp.setToZero(soleFrame);
         soleFramePointTemp.changeFrame(worldFrame);
         supportPolygon.setFootstep(robotQuadrant, soleFramePointTemp);
      }
   }

   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public ReferenceFrame getFrameByName(QuadrupedJointName name)
   {
      return namedReferenceFrames.get(name);
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
   public ReferenceFrame getRootJointFrame()
   {
      return rootJointFrame;
   }

   @Override
   public ReferenceFrame getFrameBeforeLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return framesBeforeLegJoint.get(QuadrupedJointName.getName(robotQuadrant, legJointName));
   }

   public ReferenceFrame getLegJointFrame(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return framesAfterLegJoint.get(QuadrupedJointName.getName(robotQuadrant, legJointName));
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
      return getLegJointFrame(robotQuadrant, LegJointName.KNEE_PITCH);
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

   @Override
   public ReferenceFrame getCenterOfMassZUpFrame()
   {
      return centerOfMassZUpFrame;
   }

   @Override
   public QuadrantDependentList<ReferenceFrame> getFootReferenceFrames()
   {
      return soleFrames;
   }

   @Override
   public ReferenceFrame getCenterOfFourHipsFrame()
   {
      return centerOfFourHipsFrame;
   }

   public ReferenceFrame getCenterOfFourFeetFrame()
   {
      return centerOfFourFeetFrameWithBodyRotation;
   }

   /**
    * returns the center of the support polygon excluding the specified leg
    * averaging the lowest front and the lowest hind Z values,
    * and using the nominal yaw, pitch, and roll
    * @param feetQuadrants, feet
    */
   @Override
   public ReferenceFrame getTripleSupportFrameAveragingLowestZHeightsAcrossEnds(RobotQuadrant footToExclude)
   {
      return tripleSupportFrames.get(footToExclude);
   }


   /**
    * returns the center of the polygon made up using the four feet,
    * averaging the lowest front and the lowest hind Z values,
    * and using the nominal yaw, pitch, and roll
    */
   @Override
   public ReferenceFrame getCenterOfFeetFrameAveragingLowestZHeightsAcrossEnds()
   {
      return supportPolygonCentroidFrameWithNominalRotation;
   }

   /**
    * returns the center of the four foot polygon,
    * averaging the lowest front and the lowest hind Z values,
    * and using the nominal yaw
    */
   @Override
   public ReferenceFrame getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds()
   {
      return supportPolygonCentroidZUpFrame;
   }
}
