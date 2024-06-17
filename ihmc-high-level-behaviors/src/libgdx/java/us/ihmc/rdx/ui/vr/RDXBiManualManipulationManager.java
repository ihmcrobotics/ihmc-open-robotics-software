package us.ihmc.rdx.ui.vr;

import controller_msgs.msg.dds.BimanualManipulationMessage;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXBiManualManipulationManager
{

   // TO DO : FIELDS NEED TO BE SET FROM THE UI

   private boolean enableBiManualManipulation = true;
   /**
    * Mass of the object being manipulated
    */
   public double object_mass_ = -1.0;
   /**
    * Lateral squeeze force while manipulating the object. Should approximately be squeeze_force = 0.5 * g * object_mass / friction_coefficient
    */
   public double squeeze_force_ = -1.0;
   /**
    * The squeeze and mass compensation forces ramp up over this duration (if negative a default value is used)
    */
   public double initialize_duration_ = -1.0;
   /**
    * Tracking error - if the distance between the hands varies from their initial distance by more than this value, the manipulation manager stops
    */
   public double acceptable_tracking_error_ = -1.0;

   private final SideDependentList<MovingReferenceFrame> handControlFrames = new SideDependentList<>();
   private final SideDependentList<FramePoint3D> handPoints = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final PoseReferenceFrame midHandFrame = new PoseReferenceFrame("rdxMidHandFrame", ReferenceFrame.getWorldFrame());;
   private final FramePose3D midHandFramePose = new FramePose3D();
   private final RotationMatrix midHandFrameOrientation = new RotationMatrix();
   private final FrameVector3D midHandFrameX = new FrameVector3D();
   private final FrameVector3D midHandFrameY = new FrameVector3D();
   private final FrameVector3D midHandFrameZ = new FrameVector3D();
   private final FrameVector3D worldFrameX = new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.X);
   private double nominalHandDistance;
   private boolean saveHandDistanceOnFirstTick;

   public void toggleBiManualManipulationMode()
   {
      enableBiManualManipulation = !enableBiManualManipulation;
      if (enableBiManualManipulation)
      {
         saveHandDistanceOnFirstTick = true;
      }
   }

   public BimanualManipulationMessage getBiManualManipulationMessage()
   {
      BimanualManipulationMessage message = new BimanualManipulationMessage();
      message.setDisable(!enableBiManualManipulation);
      return message;
   }

   public void setEnableBiManualManipulationMode(boolean enableBiManualManipulation)
   {
      this.enableBiManualManipulation = enableBiManualManipulation;
   }

   public boolean getEnableBiManualManipulationMode()
   {
      return enableBiManualManipulation;
   }

   public void adjustHandControlFramesForHoldingBox(SideDependentList<MutableReferenceFrame> handDesiredControlFrames)
   {
//      for (RobotSide robotSide : RobotSide.values)
//      {
//         handPoints.get(robotSide).set(ikControlFramePoses.get(robotSide).getPosition());
//      }
      updateFrames(handDesiredControlFrames);
   }

   private void updateFrames(SideDependentList<MutableReferenceFrame> handDesiredControlFrames)
   {
      if(saveHandDistanceOnFirstTick)
      {
         saveNominalHandDistanceAtFirstTick(handDesiredControlFrames);
      }
      midHandFramePose.setToZero(ReferenceFrame.getWorldFrame());
      midHandFramePose.getPosition().interpolate(handPoints.get(RobotSide.LEFT), handPoints.get(RobotSide.RIGHT), 0.5);

      midHandFrameY.sub(handPoints.get(RobotSide.LEFT), handPoints.get(RobotSide.RIGHT));
      midHandFrameY.normalize();

      midHandFrameZ.cross(worldFrameX, midHandFrameY);
      midHandFrameZ.normalize();

      midHandFrameX.cross(midHandFrameY, midHandFrameZ);
      midHandFrameOrientation.setColumns(midHandFrameX, midHandFrameY, midHandFrameZ);
      midHandFramePose.getOrientation().set(midHandFrameOrientation);

      midHandFrame.setPoseAndUpdate(midHandFramePose);

      for (RobotSide robotSide : RobotSide.values)
      {
         handDesiredControlFrames.get(robotSide).getTransformToParent().getRotation().set(midHandFramePose.getOrientation());

         handDesiredControlFrames.get(robotSide).getTransformToParent().getRotation().set(midHandFramePose.getOrientation());
         handDesiredControlFrames.get(robotSide).setParentFrame(midHandFrame);
         handDesiredControlFrames.get(robotSide).getTransformToParent().setTranslationToZero();
         handDesiredControlFrames.get(robotSide).getTransformToParent().appendTranslation(0.0, robotSide.negateIfRightSide(nominalHandDistance/2), 0.0);
         handDesiredControlFrames.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   private void saveNominalHandDistanceAtFirstTick(SideDependentList<MutableReferenceFrame> handDesiredControlFrames)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoints.get(robotSide).setMatchingFrame(handDesiredControlFrames.get(robotSide).getReferenceFrame(), 0.0, 0.0, 0.0);
      }
      nominalHandDistance = handPoints.get(RobotSide.LEFT).distance(handPoints.get(RobotSide.RIGHT));
      saveHandDistanceOnFirstTick = false;
   }
}
