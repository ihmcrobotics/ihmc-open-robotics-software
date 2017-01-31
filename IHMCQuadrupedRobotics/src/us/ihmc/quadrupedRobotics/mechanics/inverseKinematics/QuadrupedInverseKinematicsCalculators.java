package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class QuadrupedInverseKinematicsCalculators implements QuadrupedLegInverseKinematicsCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame rootJointFrame, bodyFrame;
   protected final OneDoFJoint[] oneDoFJoints;
   private final QuadrantDependentList<QuadrantHolder> quadrantHolders = new QuadrantDependentList<QuadrantHolder>();
   private final double[] jointAnglesToPack = new double[3];

   private YoGraphicReferenceFrame bodyGraphicReferenceFrame, rootJointGraphicReferenceFrame;

   public QuadrupedInverseKinematicsCalculators(QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties,
         FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      fullRobotModel.updateFrames();
      rootJointFrame = referenceFrames.getRootJointFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         QuadrantHolder quadrantHolder = new QuadrantHolder(robotQuadrant, modelFactory, physicalProperties, referenceFrames, fullRobotModel,
                                                            yoGraphicsListRegistry);

         quadrantHolders.set(robotQuadrant, quadrantHolder);
      }

      if (yoGraphicsListRegistry != null)
      {
         bodyGraphicReferenceFrame = new YoGraphicReferenceFrame(bodyFrame, registry, 0.22);
         rootJointGraphicReferenceFrame = new YoGraphicReferenceFrame(rootJointFrame, registry, 0.2);
         yoGraphicsListRegistry.registerYoGraphic("bodyGraphicReferenceFrame", bodyGraphicReferenceFrame);
         yoGraphicsListRegistry.registerYoGraphic("rootJointGraphicReferenceFrame", rootJointGraphicReferenceFrame);
      }

      fullRobotModel.updateFrames();

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean solveForEndEffectorLocationInBodyAndUpdateDesireds(RobotQuadrant robotQuadrant, Vector3d footPositionInFrameBeforeHipRoll,
         FullRobotModel fullRobotModel)
   {
      QuadrantHolder quadrantHolder = quadrantHolders.get(robotQuadrant);
      boolean validSolution = quadrantHolder.solveGivenFootLocationInHip(footPositionInFrameBeforeHipRoll, jointAnglesToPack);
      quadrantHolder.updateFrames();

      bodyGraphicReferenceFrame.update();
      rootJointGraphicReferenceFrame.update();

      //      if(validSolution)
      {
         setDesiredLegAnglesInFullRobotModel(robotQuadrant, jointAnglesToPack);
      }
      return validSolution;
   }

   public double getKneeAngleAtMaxLength(RobotQuadrant robotQuadrant)
   {
      return quadrantHolders.get(robotQuadrant).getKneeAngleAtMaxLength();
   }

   public void setLegAnglesInFullRobotModel(RobotQuadrant robotQuadrant, double[] jointAnglesToPack)
   {
      quadrantHolders.get(robotQuadrant).setLegAnglesInFullRobotModel(jointAnglesToPack);
   }

   public void setDesiredLegAnglesInFullRobotModel(RobotQuadrant robotQuadrant, double[] jointAnglesToPack)
   {
      quadrantHolders.get(robotQuadrant).setDesiredLegAnglesInFullRobotModel(jointAnglesToPack);
   }


   private class QuadrantHolder
   {
      private YoGraphicReferenceFrame attachmentGraphicReferenceFrame, hipJointGraphicReferenceFrame, kneeGraphicReferenceFrame, soleGraphicReferenceFrame,
            desiredGraphicReferenceFrame;
      private ReferenceFrame legAttachmentFrame, frameAtHip, frameAtKnee, soleFrame;

      private final QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator closedFormInverseKinematicsCalculator;

      private final FullRobotModel fullRobotModel;
      private final ArrayList<OneDoFJoint> jointsToControl = new ArrayList<OneDoFJoint>();
      private TranslationReferenceFrame desiredFrame;

      private final QuadrupedReferenceFrames referenceFrames;

      public QuadrantHolder(RobotQuadrant robotQuadrant, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties,
            QuadrupedReferenceFrames referenceFrames, FullQuadrupedRobotModel fullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
      {
         this.referenceFrames = referenceFrames;

         OneDoFJoint oneDoFJointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);

         this.fullRobotModel = fullRobotModel;
         fullRobotModel.getOneDoFJointsFromRootToHere(oneDoFJointBeforeFoot, jointsToControl);

         closedFormInverseKinematicsCalculator = QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator.createFromLegAttachmentFrame(robotQuadrant,
                                                                                                                                        modelFactory,
                                                                                                                                        physicalProperties);
         if (robotQuadrant.getEnd() == RobotEnd.FRONT)
         {
            closedFormInverseKinematicsCalculator.setBendKneesIn(true);
         }
         else
         {
            closedFormInverseKinematicsCalculator.setBendKneesIn(false);
         }

         legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);
         frameAtHip = referenceFrames.getHipPitchFrame(robotQuadrant);
         frameAtKnee = referenceFrames.getKneeFrame(robotQuadrant);

         soleFrame = referenceFrames.getFootFrame(robotQuadrant);
         desiredFrame = new TranslationReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "desiredReferenceFrame", legAttachmentFrame);

         fullRobotModel.updateFrames();
         referenceFrames.updateFrames();

         if (yoGraphicsListRegistry != null)
         {
            attachmentGraphicReferenceFrame = new YoGraphicReferenceFrame(legAttachmentFrame, registry, 0.2);
            hipJointGraphicReferenceFrame = new YoGraphicReferenceFrame(frameAtHip, registry, 0.18);
            kneeGraphicReferenceFrame = new YoGraphicReferenceFrame(frameAtKnee, registry, 0.16);
            soleGraphicReferenceFrame = new YoGraphicReferenceFrame(soleFrame, registry, 0.12);
            desiredGraphicReferenceFrame = new YoGraphicReferenceFrame(desiredFrame, registry, 0.1);

            yoGraphicsListRegistry.registerYoGraphic("attachmentGraphicReferenceFrame", attachmentGraphicReferenceFrame);
            yoGraphicsListRegistry.registerYoGraphic("hipJointGraphicReferenceFrame", hipJointGraphicReferenceFrame);
            yoGraphicsListRegistry.registerYoGraphic("kneeGraphicReferenceFrame", kneeGraphicReferenceFrame);
            yoGraphicsListRegistry.registerYoGraphic("soleGraphicReferenceFrame", soleGraphicReferenceFrame);
            yoGraphicsListRegistry.registerYoGraphic("desiredGraphicReferenceFrame", desiredGraphicReferenceFrame);
         }
      }

      public double getKneeAngleAtMaxLength()
      {
         return closedFormInverseKinematicsCalculator.getKneeAngleAtMaxLength();
      }

      public void setLegAnglesInFullRobotModel(double[] jointAnglesToPack)
      {
         for (int i = 0; i < jointsToControl.size(); i++)
         {
            jointsToControl.get(i).setQ(jointAnglesToPack[i]);
         }
      }

      public void setDesiredLegAnglesInFullRobotModel(double[] jointAnglesToPack)
      {
         for (int i = 0; i < jointsToControl.size(); i++)
         {
            jointsToControl.get(i).setqDesired(jointAnglesToPack[i]);
         }
      }

      public boolean solveGivenFootLocationInHip(Vector3d footPositionInFrameBeforeHipRoll, double[] jointAnglesToPack)
      {
         desiredFrame.updateTranslation(footPositionInFrameBeforeHipRoll);
         return closedFormInverseKinematicsCalculator.computeJointAnglesGivenFootInFrameBeforeHipRoll(footPositionInFrameBeforeHipRoll, jointAnglesToPack);
      }

      public void updateFrames()
      {
         attachmentGraphicReferenceFrame.update();
         hipJointGraphicReferenceFrame.update();
         kneeGraphicReferenceFrame.update();
         soleGraphicReferenceFrame.update();
         desiredGraphicReferenceFrame.update();
      }

      public ReferenceFrame getFootReferenceFrame()
      {
         return soleFrame;
      }
   }
}
