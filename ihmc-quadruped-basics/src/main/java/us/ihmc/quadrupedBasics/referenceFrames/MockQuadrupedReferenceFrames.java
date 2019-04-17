package us.ihmc.quadrupedBasics.referenceFrames;

import java.util.EnumMap;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class MockQuadrupedReferenceFrames extends AbstractQuadrupedReferenceFrames
{
   private final QuadrantDependentList<FramePose3D> footPoses = new QuadrantDependentList<FramePose3D>();
   private final QuadrantDependentList<PoseReferenceFrame> soleFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<MovingReferenceFrame> movingSoleFrames = new QuadrantDependentList<>();
   private final SideDependentList<ReferenceFrame> sideDependentMidFeetZUpFrames = new SideDependentList<ReferenceFrame>();
   private final EndDependentList<ReferenceFrame> endDependentMidFeetZUpFrames = new EndDependentList<ReferenceFrame>();

   public MockQuadrupedReferenceFrames()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footPoses.set(robotQuadrant, new FramePose3D(ReferenceFrame.getWorldFrame()));
         soleFrames.set(robotQuadrant,
               new PoseReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "soleFrame", footPoses.get(robotQuadrant)));
         movingSoleFrames.set(robotQuadrant, MovingReferenceFrame.constructFrameFixedInParent(robotQuadrant.getCamelCaseName() + "MovingSoleFrame",
                                                                                              soleFrames.get(robotQuadrant), new RigidBodyTransform()));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant hindSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide);
         RobotQuadrant frontSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide);

         MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame(robotSide.getCamelCaseNameForStartOfExpression() + "MidFeetZUpFrame",
               ReferenceFrame.getWorldFrame(), soleFrames.get(hindSoleQuadrant), soleFrames.get(frontSoleQuadrant));
         sideDependentMidFeetZUpFrames.put(robotSide, midFeetZUpFrame);
      }

      for (RobotEnd robotEnd : RobotEnd.values)
      {
         RobotQuadrant leftSoleQuadrant = RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT);
         RobotQuadrant rightSoleQuadrant = RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT);

         MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame(robotEnd.getCamelCaseNameForStartOfExpression() + "MidFeetZUpFrame",
               ReferenceFrame.getWorldFrame(), soleFrames.get(leftSoleQuadrant), soleFrames.get(rightSoleQuadrant));
         endDependentMidFeetZUpFrames.put(robotEnd, midFeetZUpFrame);
      }
      
      initializeCommonValues();
   }

   public void update(QuadrantDependentList<YoFramePoint3D> yoFootPositions)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePose3D footPose = footPoses.get(robotQuadrant);
         footPose.setPosition(yoFootPositions.get(robotQuadrant));
         soleFrames.get(robotQuadrant).setPoseAndUpdate(footPose);
         movingSoleFrames.get(robotQuadrant).update();
      }
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         endDependentMidFeetZUpFrames.get(robotEnd).update();
      }
      for (RobotSide robotSide : RobotSide.values)
      {
         sideDependentMidFeetZUpFrames.get(robotSide).update();
      }
   }

   @Override
   public ReferenceFrame getSideDependentMidFeetZUpFrame(RobotSide robotSide)
   {
      return sideDependentMidFeetZUpFrames.get(robotSide);
   }

   @Override
   public ReferenceFrame getRootJointFrame()
   {
      return null;
   }

   @Override
   public ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant)
   {
      return null;
   }

   @Override
   public ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant)
   {
      return null;
   }

   @Override
   public ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant)
   {
      return null;
   }

   @Override
   public ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant)
   {
      return null;
   }

   @Override
   public SegmentDependentList<RobotQuadrant, MovingReferenceFrame> getAnkleZUpReferenceFrames()
   {
      return null;
   }

   @Override
   public SegmentDependentList<RobotQuadrant, MovingReferenceFrame> getFootReferenceFrames()
   {
      return null;
   }

   @Override
   public SegmentDependentList<RobotQuadrant, MovingReferenceFrame> getSoleFrames()
   {
      return movingSoleFrames;
   }

   @Override
   public MovingReferenceFrame getAnkleZUpFrame(RobotQuadrant robotSegment)
   {
      return null;
   }

   @Override
   public MovingReferenceFrame getFootFrame(RobotQuadrant robotQuadrant)
   {
      return null;
   }

   @Override
   public MovingReferenceFrame getLegJointFrame(RobotQuadrant robotSegment, LegJointName legJointName)
   {
      return null;
   }

   @Override
   public EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(RobotQuadrant robotSegment)
   {
      return null;
   }

   @Override
   public ReferenceFrame getIMUFrame()
   {
      return null;
   }

   @Override
   public MovingReferenceFrame getSoleFrame(RobotQuadrant robotSegment)
   {
      return movingSoleFrames.get(robotSegment);
   }

   @Override
   public MovingReferenceFrame getSoleZUpFrame(RobotQuadrant robotSegment)
   {
      return null;
   }

   @Override
   public SegmentDependentList<RobotQuadrant, MovingReferenceFrame> getSoleZUpFrames()
   {
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfMassZUpFrame()
   {
      return null;
   }

   @Override
   public ReferenceFrame getBodyZUpFrame()
   {
      return null;
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfFourHipsFrame()
   {
      return null;
   }

   @Override
   public ReferenceFrame getFrameBeforeLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return null;
   }

   @Override
   public void updateFrames()
   {
      
   }

   @Override
   public ReferenceFrame getTripleSupportFrameAveragingLowestZHeightsAcrossEnds(RobotQuadrant footToExclude)
   {
      return null;
   }


   @Override
   public ReferenceFrame getCenterOfFeetFrameAveragingLowestZHeightsAcrossEnds()
   {
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds()
   {
      return null;
   }
}