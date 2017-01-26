package us.ihmc.exampleSimulations.beetle.referenceFrames;

import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class HexapodReferenceFrames implements ReferenceFrames
{
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final TranslationReferenceFrame centerOfFeetFrame;
   private final SegmentDependentList<RobotSextant, ReferenceFrame> feetFrames = new SegmentDependentList<>(RobotSextant.class);
   private final FullRobotModel fullRobotModel;
   private final ZUpFrame bodyZUpFrame;
   private final PoseReferenceFrame centerOfMassFrameWithBodyZUpOrientation;
   private final FramePoint centerOfMassPosition = new FramePoint();
   private final FrameOrientation bodyOrientation = new FrameOrientation();

   private final FramePoint centroid = new FramePoint();
   private final FramePoint currentFoot = new FramePoint();

   public HexapodReferenceFrames(FullRobotModel fullRobotModel, SegmentDependentList<RobotSextant, Vector3d> offsetsFromJointBeforeFootToSole)
   {
      this.fullRobotModel = fullRobotModel;
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      bodyZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), fullRobotModel.getPelvis().getBodyFixedFrame(), "bodyZUpFrame");
      centerOfMassFrameWithBodyZUpOrientation = new PoseReferenceFrame("centerOfMassFrameWithBodyOrientation", ReferenceFrame.getWorldFrame());

      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RigidBody endEffector = fullRobotModel.getEndEffector(robotSextant);
         InverseDynamicsJoint parentJoint = endEffector.getParentJoint();
         ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
         TranslationReferenceFrame footFrame = new TranslationReferenceFrame(robotSextant.name() + "footFrame", frameAfterJoint);
         Vector3d offsetFromJointBeforeFootToSole = offsetsFromJointBeforeFootToSole.get(robotSextant);
         footFrame.updateTranslation(offsetFromJointBeforeFootToSole);
         feetFrames.set(robotSextant, footFrame);
      }

      centerOfFeetFrame = new TranslationReferenceFrame("centerOfFeetFrame", ReferenceFrame.getWorldFrame());
      updateCenterOfFeetFrame();
   }

   private void updateCenterOfFeetFrame()
   {
      centroid.setToZero();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         currentFoot.setToZero(getFootFrame(robotSextant));
         currentFoot.changeFrame(ReferenceFrame.getWorldFrame());
         centroid.add(currentFoot);
      }

      centroid.scale(1.0 / 6.0);
      centroid.setZ(0.0);
      centerOfFeetFrame.updateTranslation(centroid);
   }

   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public ReferenceFrame getCenterOfFeetFrame()
   {
      return centerOfFeetFrame;

   }

   public ReferenceFrame getFootFrame(RobotSextant robotSextant)
   {
      return feetFrames.get(robotSextant);
   }

   public void updateFrames()
   {
      fullRobotModel.updateFrames();
      centerOfMassFrame.update();
      bodyZUpFrame.update();
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(ReferenceFrame.getWorldFrame());
      bodyOrientation.setToZero(bodyZUpFrame);
      bodyOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      getCenterOfMassFrameWithBodyZUpOrientation().setPoseAndUpdate(centerOfMassPosition, bodyOrientation);
      updateCenterOfFeetFrame();
   }

   public ReferenceFrame getBodyZUpFrame()
   {
      return bodyZUpFrame;
   }

   public PoseReferenceFrame getCenterOfMassFrameWithBodyZUpOrientation()
   {
      return centerOfMassFrameWithBodyZUpOrientation;
   }
}
