package us.ihmc.quadrupedRobotics.estimator.referenceFrames;

import us.ihmc.robotics.partNames.LegJointName;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class MockQuadrupedReferenceFrames extends CommonQuadrupedReferenceFrames
{
   private final QuadrantDependentList<FramePose> footPoses = new QuadrantDependentList<FramePose>();
   private final QuadrantDependentList<PoseReferenceFrame> soleFrames = new QuadrantDependentList<PoseReferenceFrame>();
   private final SideDependentList<ReferenceFrame> sideDependentMidFeetZUpFrames = new SideDependentList<ReferenceFrame>();
   private final EndDependentList<ReferenceFrame> endDependentMidFeetZUpFrames = new EndDependentList<ReferenceFrame>();

   public MockQuadrupedReferenceFrames()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footPoses.set(robotQuadrant, new FramePose(ReferenceFrame.getWorldFrame()));
         soleFrames.set(robotQuadrant,
               new PoseReferenceFrame(robotQuadrant.getCamelCaseNameForStartOfExpression() + "soleFrame", footPoses.get(robotQuadrant)));
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

   public void update(QuadrantDependentList<YoFramePoint> yoFootPositions)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePose footPose = footPoses.get(robotQuadrant);
         footPose.setPosition(yoFootPositions.get(robotQuadrant).getFramePointCopy());
         soleFrames.get(robotQuadrant).setPoseAndUpdate(footPose);
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
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getLegAttachmentFrame(RobotQuadrant robotQuadrant)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getKneeFrame(RobotQuadrant robotQuadrant)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getHipRollFrame(RobotQuadrant robotQuadrant)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getHipPitchFrame(RobotQuadrant robotQuadrant)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public QuadrantDependentList<ReferenceFrame> getFootReferenceFrames()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getFootFrame(RobotQuadrant robotQuadrant)
   {
      return soleFrames.get(robotQuadrant);
   }
   
   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfMassZUpFrame()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getBodyZUpFrame()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfFourHipsFrame()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getFrameBeforeLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void updateFrames()
   {
      
   }

   @Override
   public ReferenceFrame getTripleSupportFrameAveragingLowestZHeightsAcrossEnds(RobotQuadrant footToExclude)
   {
      // TODO Auto-generated method stub
      return null;
   }


   @Override
   public ReferenceFrame getCenterOfFeetFrameAveragingLowestZHeightsAcrossEnds()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public TLongObjectHashMap<ReferenceFrame> getReferenceFrameDefaultHashIds()
   {
      // TODO Auto-generated method stub
      return null;
   }


}