package us.ihmc.quadrupedBasics.referenceFrames;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotModels.FullLeggedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;

public class QuadrupedReferenceFrames extends AbstractQuadrupedReferenceFrames
{
   private final FullLeggedRobotModel<RobotQuadrant> fullRobotModel;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Quaternion IDENTITY_QUATERNION = new Quaternion();

   private final MovingReferenceFrame bodyFrame, rootJointFrame;
   private final MovingZUpFrame bodyZUpFrame;

   private final Map<QuadrupedJointName, MovingReferenceFrame> framesBeforeLegJoint = new EnumMap<>(QuadrupedJointName.class);
   private final QuadrantDependentList<EnumMap<LegJointName, MovingReferenceFrame>> legFrames = QuadrantDependentList.createListOfEnumMaps(LegJointName.class);

   private final QuadrantDependentList<MovingReferenceFrame> ankleZUpFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<MovingReferenceFrame> soleZUpFrames = new QuadrantDependentList<>();

   private final QuadrantDependentList<PoseReferenceFrame> tripleSupportFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePose3D> tripleSupportCentroidPoses = new QuadrantDependentList<>();

   private final FramePose3D supportPolygonCentroidWithNominalRotation = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame supportPolygonCentroidFrameWithNominalRotation;
   private final ZUpFrame supportPolygonCentroidZUpFrame;

   private final QuadrantDependentList<ReferenceFrame> legAttachementFrames = new QuadrantDependentList<ReferenceFrame>();
   private final QuadrantDependentList<FramePoint3D> legAttachementPoints= new QuadrantDependentList<FramePoint3D>();

   private final SideDependentList<ReferenceFrame> sideDependentMidFeetZUpFrames = new SideDependentList<ReferenceFrame>();

   private final ReferenceFrame centerOfMassFrame;
   private final PoseReferenceFrame centerOfMassFrameWithRotation;
   private final ZUpFrame centerOfMassZUpFrame;
   private final FramePose3D centerOfMassPose;
   private final PoseReferenceFrame centerOfFourHipsFrame;
   private final FramePose3D centerOfFourHipsFramePose;
   private final FramePoint3D centerOfFourHipsFramePoint = new FramePoint3D();

   private final PoseReferenceFrame centerOfFourFeetFrameWithBodyRotation;
   private final FramePose3D centerOfFourFeetFramePose;
   private final FramePoint3D centerOfFourFeetFramePoint = new FramePoint3D();

   private QuadrupedSupportPolygon supportPolygonForCentroids = new QuadrupedSupportPolygon();

   public QuadrupedReferenceFrames(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;

      rootJointFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      bodyFrame = fullRobotModel.getRootJoint().getSuccessor().getBodyFixedFrame();
      centerOfMassPose = new FramePose3D(bodyFrame);

      bodyZUpFrame = new MovingZUpFrame(bodyFrame, "bodyZUpFrame");

      RobotSpecificJointNames robotJointNames = fullRobotModel.getRobotSpecificJointNames();
      if (robotJointNames.getLegJointNames() != null)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            for (LegJointName legJointName : robotJointNames.getLegJointNames())
            {
               MovingReferenceFrame legJointFrame = fullRobotModel.getFrameAfterLegJoint(robotQuadrant, legJointName);
               legFrames.get(robotQuadrant).put(legJointName, legJointFrame);
            }
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (OneDoFJointBasics oneDoFJoint : fullRobotModel.getLegJointsList(robotQuadrant))
         {
            MovingReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
            QuadrupedJointName legJointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
            framesBeforeLegJoint.put(legJointName, frameBeforeJoint);
         }

         MovingReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotQuadrant);
         soleFrames.put(robotQuadrant, soleFrame);

         MovingZUpFrame soleZUpFrame = new MovingZUpFrame(soleFrame, soleFrame.getName() + "ZUp");
         soleZUpFrames.put(robotQuadrant, soleZUpFrame);

         FramePoint3D legAttachmentPoint = new FramePoint3D();
         legAttachementPoints.set(robotQuadrant, legAttachmentPoint);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant hindSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide);
         RobotQuadrant frontSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide);

         MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame(robotSide.getCamelCaseNameForStartOfExpression() + "MidFeetZUpFrame", worldFrame,
                                                                 soleZUpFrames.get(hindSoleQuadrant), soleZUpFrames.get(frontSoleQuadrant));
         sideDependentMidFeetZUpFrames.put(robotSide, midFeetZUpFrame);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedJointName hipRollJointName = QuadrupedJointName.getName(robotQuadrant, LegJointName.HIP_ROLL);
         ReferenceFrame frameBeforeHipRoll = framesBeforeLegJoint.get(hipRollJointName);

         QuadrupedJointName hipPitchJointName = QuadrupedJointName.getName(robotQuadrant, LegJointName.HIP_PITCH);
         ReferenceFrame frameBeforeHipPitch = framesBeforeLegJoint.get(hipPitchJointName);

         FramePoint3D xyOffsetFromRollToPitch = new FramePoint3D(frameBeforeHipPitch);
         xyOffsetFromRollToPitch.changeFrame(frameBeforeHipRoll);

         TranslationReferenceFrame legAttachmentFrame = new TranslationReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "LegAttachementFrame", frameBeforeHipRoll);

         xyOffsetFromRollToPitch.setZ(0.0);

         legAttachmentFrame.updateTranslation(xyOffsetFromRollToPitch);
         legAttachementFrames.set(robotQuadrant, legAttachmentFrame);

         MovingReferenceFrame footFrame = getFootFrame(robotQuadrant);

         MovingZUpFrame ankleZUpFrame = new MovingZUpFrame(footFrame, robotQuadrant.getCamelCaseNameForStartOfExpression() + "AnkleZUp");
         ankleZUpFrames.put(robotQuadrant, ankleZUpFrame);

         FramePose3D tripleSupportCentroidPose = new FramePose3D(worldFrame);
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

      centerOfFourHipsFramePose = new FramePose3D(bodyFrame);
      centerOfFourHipsFrame = new PoseReferenceFrame("centerOfFourHipsFrame", bodyFrame);

      centerOfFourFeetFramePose = new FramePose3D(bodyFrame);
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
         FramePoint3D legAttachmentPoint = legAttachementPoints.get(quadrant);
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
         legAttachementFrames.get(robotQuadrant).update();
         soleFrames.get(robotQuadrant).update();
         soleZUpFrames.get(robotQuadrant).update();
         ankleZUpFrames.get(robotQuadrant).update();
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
         FramePose3D framePose = tripleSupportCentroidPoses.get(swingLeg);
         supportPolygonForCentroids.getWeightedCentroidFramePoseAveragingLowestZHeightsAcrossEnds(framePose);

         PoseReferenceFrame tripleSupportFrame = tripleSupportFrames.get(swingLeg);
         tripleSupportFrame.setPoseAndUpdate(framePose);
      }
   }

   private void updateCenterOfFeetUsingNominalsForRotationPart()
   {
      updateSupportPolygon(null, supportPolygonForCentroids);
      // only update if getting the centroid was successful.
      if (supportPolygonForCentroids.getCentroidFramePoseAveragingLowestZHeightsAcrossEnds(supportPolygonCentroidWithNominalRotation))
         supportPolygonCentroidFrameWithNominalRotation.setPoseAndUpdate(supportPolygonCentroidWithNominalRotation);
      supportPolygonCentroidZUpFrame.update();
   }

   private final FramePoint3D soleFramePointTemp = new FramePoint3D();
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

   @Override
   public MovingReferenceFrame getBodyFrame()
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

   @Override
   public MovingReferenceFrame getLegJointFrame(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return legFrames.get(robotQuadrant).get(legJointName);
   }

   @Override
   public EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(RobotQuadrant robotQuadrant)
   {
      return legFrames.get(robotQuadrant);
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
   public MovingReferenceFrame getFootFrame(RobotQuadrant robotQuadrant)
   {
      return fullRobotModel.getEndEffectorFrame(robotQuadrant, LimbName.LEG);
   }

   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
      //return centerOfMassFrameWithRotation;
   }

   @Override
   public ReferenceFrame getCenterOfMassZUpFrame()
   {
      return centerOfMassZUpFrame;
   }

   @Override
   public QuadrantDependentList<MovingReferenceFrame> getFootReferenceFrames()
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
    * @param footToExclude, feet
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

   @Override
   public MovingReferenceFrame getIMUFrame()
   {
      return bodyFrame;
   }

   @Override
   public MovingReferenceFrame getSoleFrame(RobotQuadrant robotQuadrant)
   {
      return soleFrames.get(robotQuadrant);
   }

   @Override
   public QuadrantDependentList<MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   @Override
   public MovingReferenceFrame getSoleZUpFrame(RobotQuadrant robotQuadrant)
   {
      return soleZUpFrames.get(robotQuadrant);
   }

   @Override
   public QuadrantDependentList<MovingReferenceFrame> getSoleZUpFrames()
   {
      return soleZUpFrames;
   }

   @Override
   public MovingReferenceFrame getAnkleZUpFrame(RobotQuadrant robotQuadrant)
   {
      return ankleZUpFrames.get(robotQuadrant);
   }

   @Override
   public QuadrantDependentList<MovingReferenceFrame> getAnkleZUpReferenceFrames()
   {
      return ankleZUpFrames;
   }
}
