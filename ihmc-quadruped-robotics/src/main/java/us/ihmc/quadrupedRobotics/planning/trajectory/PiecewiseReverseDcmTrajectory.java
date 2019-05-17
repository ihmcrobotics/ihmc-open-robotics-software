package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class PiecewiseReverseDcmTrajectory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean visualize = false;

   private boolean initialized;
   private final int maxSteps;
   private int numberOfSteps;
   private final double gravity;
   private double comHeight;
   private final double[] timesAtStartOfSteps;
   private final List<YoFramePoint3D> dcmCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpCornerPoints = new ArrayList<>();

   private final FramePoint3D dcmPosition = new FramePoint3D();
   private final FramePoint3D dcmPositionAtEndOfSwing = new FramePoint3D();
   private final FrameVector3D dcmVelocity = new FrameVector3D();

   private final List<MutableDouble> temporaryDouble;
   private final List<FramePoint3D> temporaryFramePoints;

   public PiecewiseReverseDcmTrajectory(int maxSteps, double gravity, double comHeight, YoVariableRegistry registry)
   {
      if (maxSteps < 1)
         throw new RuntimeException("maxSteps must be greater than 0");

      this.initialized = false;
      this.maxSteps = maxSteps;
      this.gravity = gravity;
      this.comHeight = Math.max(comHeight, 0.001);

      numberOfSteps = maxSteps;
      timesAtStartOfSteps = new double[maxSteps + 1];
      for (int i = 0; i < maxSteps + 1; i++)
      {
         dcmCornerPoints.add(new YoFramePoint3D("dcmCornerPoint" + i, worldFrame, registry));
         vrpCornerPoints.add(new YoFramePoint3D("vrpCornerPoint" + i, worldFrame, registry));
      }
      temporaryDouble = new ArrayList<>();
      temporaryDouble.add(new MutableDouble(0));
      temporaryFramePoints = new ArrayList<>();
      temporaryFramePoints.add(new FramePoint3D());

      resetVariables();
   }

   public void resetVariables()
   {
      for (int i = 0; i < dcmCornerPoints.size(); i ++)
      {
         dcmCornerPoints.get(i).setToNaN();
         vrpCornerPoints.get(i).setToNaN();
      }
   }

   public void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, double pointSize)
   {
      YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      for (int i = 0; i < dcmCornerPoints.size(); i++)
      {
         YoFramePoint3D dcmCornerPoint = dcmCornerPoints.get(i);
         YoFramePoint3D vrpCornerPoint = vrpCornerPoints.get(i);
         YoGraphicPosition dcmCornerPointViz = new YoGraphicPosition("DCMCornerPoint" + i, dcmCornerPoint, pointSize, YoAppearance.Blue(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         YoGraphicPosition vrpCornerPointViz = new YoGraphicPosition("VRPCornerPoint" + i, vrpCornerPoint, pointSize, YoAppearance.Blue(),
                                                                     YoGraphicPosition.GraphicType.SOLID_BALL);
         graphicsList.add(dcmCornerPointViz);
         graphicsList.add(vrpCornerPointViz);

         artifactList.add(dcmCornerPointViz.createArtifact());
         artifactList.add(vrpCornerPointViz.createArtifact());
      }

      graphicsList.setVisible(visualize);
      artifactList.setVisible(visualize);

      yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /**
    * Computes a piecewise DCM trajectory assuming a constant CMP during each step. The DCM dynamics
    * are integrated in reverse time given a desired final DCM position at the end of the final step.
    *
    * @param numberOfSteps number of steps
    * @param timesAtStartOfSteps time at the start of each step
    * @param cmpPositionAtStartOfSteps centroidal moment pivot position at the start of each step
    * @param finalTime time at the end of the final step
    * @param finalDcmPosition divergent component of motion position at the end of the final step
    */
   public void initializeTrajectory(int numberOfSteps, List<MutableDouble> timesAtStartOfSteps, List<? extends FramePoint3DReadOnly> cmpPositionAtStartOfSteps,
                                    double finalTime, FramePoint3DReadOnly finalDcmPosition)
   {
      resetVariables();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      if ((maxSteps < numberOfSteps) || (timesAtStartOfSteps.size() < numberOfSteps) || (cmpPositionAtStartOfSteps.size() < numberOfSteps))
      {
         throw new RuntimeException("number of steps exceeds the maximum buffer size");
      }
      this.numberOfSteps = numberOfSteps;

      // compute dcm position at start of each step assuming a piecewise constant vrp trajectory
      for (int i = 0; i < numberOfSteps; i++)
      {
         this.timesAtStartOfSteps[i] = timesAtStartOfSteps.get(i).doubleValue();
         vrpCornerPoints.get(i).setMatchingFrame(cmpPositionAtStartOfSteps.get(i));
         vrpCornerPoints.get(i).addZ(comHeight);
      }
      this.timesAtStartOfSteps[numberOfSteps] = finalTime;

      dcmCornerPoints.get(numberOfSteps).setMatchingFrame(finalDcmPosition);

      for (int i = numberOfSteps - 1; i >= 0; i--)
      {
         dcmCornerPoints.get(i).set(dcmCornerPoints.get(i + 1));
         dcmCornerPoints.get(i).sub(vrpCornerPoints.get(i));
         dcmCornerPoints.get(i).scale(Math.exp(-naturalFrequency * (this.timesAtStartOfSteps[i + 1] - this.timesAtStartOfSteps[i])));
         dcmCornerPoints.get(i).add(vrpCornerPoints.get(i));
      }
      this.initialized = true;
      computeTrajectory(this.timesAtStartOfSteps[0]);
   }


   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between steps
      currentTime = Math.min(Math.max(currentTime, timesAtStartOfSteps[0]), timesAtStartOfSteps[numberOfSteps]);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      for (int i = numberOfSteps - 1; i >= 0; i--)
      {
         if (currentTime >= timesAtStartOfSteps[i])
         {
            double exponential = Math.exp(naturalFrequency * (currentTime - timesAtStartOfSteps[i]));
            dcmPositionAtEndOfSwing.set(dcmCornerPoints.get(i + 1));
            dcmPosition.interpolate(vrpCornerPoints.get(i), dcmCornerPoints.get(i), exponential);
            dcmVelocity.sub(dcmPosition, vrpCornerPoints.get(i));
            dcmVelocity.scale(naturalFrequency);
            break;
         }
      }
   }

   public double getNaturalFrequency()
   {
      return Math.sqrt(gravity / comHeight);
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = Math.max(comHeight, 0.001);
   }

   public double getStartTime()
   {
      return timesAtStartOfSteps[0];
   }

   public void getPosition(FramePoint3D dcmPositionToPack)
   {
      dcmPositionToPack.setIncludingFrame(dcmPosition);
   }

   public void getPositionAtEndOfSwing(FramePoint3D dcmPositionToPack)
   {
      dcmPositionToPack.setIncludingFrame(dcmPositionAtEndOfSwing);
   }

   public void getVelocity(FrameVector3D dcmVelocityToPack)
   {
      dcmVelocityToPack.setIncludingFrame(dcmVelocity);
   }

   public static void main(String args[])
   {
      double comHeight = 1.0;
      double gravity = 9.81;
      YoVariableRegistry registry = new YoVariableRegistry("you");
      PiecewiseReverseDcmTrajectory dcmTrajectory = new PiecewiseReverseDcmTrajectory(10, gravity, comHeight, registry);

      List<MutableDouble> timeAtSoS = new ArrayList(2);
      timeAtSoS.add(0, new MutableDouble(0.0));
      timeAtSoS.add(1, new MutableDouble(0.4));
      List<FramePoint3D> cmpPositionAtSoS = new ArrayList<>(2);
      cmpPositionAtSoS.add(0, new FramePoint3D());
      cmpPositionAtSoS.add(1, new FramePoint3D());
      cmpPositionAtSoS.get(0).set(0.0, 0.0, 0.0);
      cmpPositionAtSoS.get(1).set(0.0, -0.4, 0.0);

      double timeAtEoS = 0.8;
      FramePoint3D dcmPositionAtEoS = new FramePoint3D(worldFrame);
      dcmPositionAtEoS.set(0.0, -0.2, comHeight);
      dcmTrajectory.initializeTrajectory(2, timeAtSoS, cmpPositionAtSoS, timeAtEoS, dcmPositionAtEoS);

      FramePoint3D dcmPosition = new FramePoint3D(worldFrame);
      for (int i = 0; i < timeAtSoS.size(); i++)
      {
         dcmTrajectory.computeTrajectory(timeAtSoS.get(i).doubleValue());
         dcmTrajectory.getPosition(dcmPosition);
         System.out.println("dcm position at start of step " + i + " : " + dcmPosition);
      }
   }
}

