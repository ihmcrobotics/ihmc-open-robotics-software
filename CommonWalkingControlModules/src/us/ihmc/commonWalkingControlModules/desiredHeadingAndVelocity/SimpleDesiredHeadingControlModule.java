package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import java.awt.Color;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.YoFrameLineSegment2dArtifact;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;

public class SimpleDesiredHeadingControlModule implements DesiredHeadingControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredHeadingControlModule");
   private final DoubleYoVariable desiredHeadingFinal = new DoubleYoVariable("desiredHeadingFinal",
                                                           "Yaw of the desired heading frame with respect to the world.", registry);
   private final DoubleYoVariable desiredHeading = new DoubleYoVariable("desiredHeading", registry);
   private final DoubleYoVariable maxHeadingDot = new DoubleYoVariable("maxHeadingDot", "In units of rad/sec", registry);

   private final DesiredHeadingFrame desiredHeadingFrame = new DesiredHeadingFrame();

   private final YoFrameLineSegment2d desiredHeadingLine;
   private final YoFrameLineSegment2d finalHeadingLine;

   private final ProcessedSensorsInterface processedSensors;
   private final double controlDT;

   public SimpleDesiredHeadingControlModule(double desiredHeadingfinal, ProcessedSensorsInterface processedSensors, double controlDT,
           YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.processedSensors = processedSensors;
      parentRegistry.addChild(registry);
      this.controlDT = controlDT;

      maxHeadingDot.set(0.1);

      this.desiredHeadingFinal.set(desiredHeadingfinal);
      this.desiredHeading.set(this.desiredHeadingFinal.getDoubleValue());    // The final is the first one according to the initial setup of the robot

      desiredHeadingLine = new YoFrameLineSegment2d("desiredHeadingLine", "", ReferenceFrame.getWorldFrame(), registry);
      finalHeadingLine = new YoFrameLineSegment2d("finalHeadingLine", "", ReferenceFrame.getWorldFrame(), registry);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("Simple Desired Heading");

         {
            YoFrameLineSegment2dArtifact yoFrameLineSegment2dArtifact = new YoFrameLineSegment2dArtifact("Desired Heading Line", desiredHeadingLine,
                                                                           Color.MAGENTA);
            artifactList.add(yoFrameLineSegment2dArtifact);
         }

         {
            YoFrameLineSegment2dArtifact yoFrameLineSegment2dArtifact = new YoFrameLineSegment2dArtifact("Final Heading Line", finalHeadingLine, Color.ORANGE);
            artifactList.add(yoFrameLineSegment2dArtifact);
         }

         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void updateDesiredHeadingFrame()
   {
      updateDesiredHeading();
      updatedDesiredHeadingLine();
      updatedFinalHeadingLine();
      desiredHeadingFrame.update();
   }

   private void updatedDesiredHeadingLine()
   {
      FramePoint2d endpoint1 = processedSensors.getCenterOfMassGroundProjectionInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d();
      FramePoint2d endpoint2 = new FramePoint2d(endpoint1);
      double length = 1.0;
      endpoint2.setX(endpoint2.getX() + length * Math.cos(desiredHeading.getDoubleValue()));
      endpoint2.setY(endpoint2.getY() + length * Math.sin(desiredHeading.getDoubleValue()));

      FrameLineSegment2d frameLineSegment2d = new FrameLineSegment2d(endpoint1, endpoint2);
      desiredHeadingLine.setFrameLineSegment2d(frameLineSegment2d);
   }

   public FrameVector getFinalHeadingTarget()
   {
      FrameVector finalHeading = new FrameVector(ReferenceFrame.getWorldFrame(), Math.cos(desiredHeadingFinal.getDoubleValue()),
                                    Math.sin(desiredHeadingFinal.getDoubleValue()), 0.0);

      return finalHeading;
   }

   public ReferenceFrame getDesiredHeadingFrame()
   {
      return desiredHeadingFrame;
   }

   public void setFinalDesiredHeading(double desiredHeading)
   {
      this.desiredHeadingFinal.set(desiredHeading);
   }

   public double getDesiredHeading()
   {
      return desiredHeading.getDoubleValue();
   }

   public void resetHeading(double newHeading)
   {
      this.desiredHeading.set(newHeading);
      this.desiredHeadingFinal.set(newHeading);
   }

   private void updatedFinalHeadingLine()
   {
      FramePoint2d endpoint1 = processedSensors.getCenterOfMassGroundProjectionInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d();
      FramePoint2d endpoint2 = new FramePoint2d(endpoint1);
      double length = 1.0;
      endpoint2.setX(endpoint2.getX() + length * Math.cos(desiredHeadingFinal.getDoubleValue()));
      endpoint2.setY(endpoint2.getY() + length * Math.sin(desiredHeadingFinal.getDoubleValue()));

      FrameLineSegment2d frameLineSegment2d = new FrameLineSegment2d(endpoint1, endpoint2);
      finalHeadingLine.setFrameLineSegment2d(frameLineSegment2d);
   }

   private void updateDesiredHeading()
   {
      double error = desiredHeadingFinal.getDoubleValue() - desiredHeading.getDoubleValue();
      double maximumChangePerTick = maxHeadingDot.getDoubleValue() * controlDT;

      double deltaHeading = MathTools.clipToMinMax(error, -maximumChangePerTick, maximumChangePerTick);

      desiredHeading.set(desiredHeading.getDoubleValue() + deltaHeading);
   }

   private class DesiredHeadingFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 4657294310129415811L;

      public DesiredHeadingFrame()
      {
         super("DesiredHeadingFrame", ReferenceFrame.getWorldFrame(), false, false, true);
      }

      public void updateTransformToParent(Transform3D transformToParent)
      {
         Matrix3d rotation = new Matrix3d();
         rotation.rotZ(desiredHeading.getDoubleValue());

         transformToParent.set(rotation);
      }
   }
}
