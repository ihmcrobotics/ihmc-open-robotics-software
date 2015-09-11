package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


public class AdjustableParabolicTrajectory extends ParabolicCartesianTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final BooleanYoVariable apexChangedSinceLastRedraw;
   private final YoFramePoint startPoint;
   private final YoFramePoint endPoint;
   private final FrameVector zeroAccelerationVector;

   private final BagOfBalls bagOfBalls;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int numberOfStepsToVisualize;
   private final ArrayList<FramePoint> listOfBallPositions;
   private final boolean adjustmentMode = true;

   public AdjustableParabolicTrajectory(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry, FramePoint startPoint,
         FramePoint endPoint, double apexOfParabola)
   {
      this("adjustableParabolicTrajectory", parentRegistry, graphicsRegistry, startPoint, endPoint, apexOfParabola, (int) endPoint.distance(startPoint) * 30,
            endPoint.distance(startPoint));
   }

   public AdjustableParabolicTrajectory(String trajectoryName, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry,
         FramePoint startPoint, FramePoint endPoint, double apexOfParabola, int numberOfStepsToVisualize, double stepTime)
   {
      super(trajectoryName, ReferenceFrame.getWorldFrame(), new ConstantDoubleProvider(stepTime), apexOfParabola, parentRegistry);
      registry = new YoVariableRegistry(trajectoryName);
      parentRegistry.addChild(registry);
      
      this.startPoint = new YoFramePoint("startPoint", worldFrame, registry);
      this.startPoint.set(startPoint);
      
      this.endPoint = new YoFramePoint("endPoint", worldFrame, registry);
      this.endPoint.set(endPoint);
      
      this.numberOfStepsToVisualize = numberOfStepsToVisualize;
      this.zeroAccelerationVector = new FrameVector(worldFrame);
      this.apexChangedSinceLastRedraw = new BooleanYoVariable("apexChangedSinceLastRedraw", this.registry);

      updateGroundClearance(apexOfParabola);

      if (!startPoint.getReferenceFrame().isWorldFrame())
         startPoint.changeFrame(worldFrame);
      if (!endPoint.getReferenceFrame().isWorldFrame())
         endPoint.changeFrame(worldFrame);

      initialize(this.startPoint.getFramePointCopy(), zeroAccelerationVector, zeroAccelerationVector, this.endPoint.getFramePointCopy(), zeroAccelerationVector);

      bagOfBalls = new BagOfBalls(this.numberOfStepsToVisualize, this.registry, graphicsRegistry);
      apexChangedSinceLastRedraw.set(true);
      listOfBallPositions = new ArrayList<FramePoint>(this.numberOfStepsToVisualize);
      updateBagOfBallsGraphics();
   }

   public void updateStartPoint(FramePoint newStartPoint)
   {
      if (!newStartPoint.getReferenceFrame().isWorldFrame())
      {
         newStartPoint.changeFrame(worldFrame);
      }

      startPoint.set(newStartPoint);
      setApex(this.groundClearance.getDoubleValue());
   }

   public void updateEndPoint(FramePoint newEndPoint)
   {
      if (!newEndPoint.getReferenceFrame().isWorldFrame())
      {
         newEndPoint.changeFrame(worldFrame);
      }

      endPoint.set(newEndPoint);
      setApex(this.groundClearance.getDoubleValue());
   }

   public void setApex(double newApex)
   {
      if(adjustmentMode)
         updateGroundClearance(newApex);
      else
         updateGroundClearance(getCurrentGroundClearance());
      initialize(startPoint.getFramePointCopy(), zeroAccelerationVector, zeroAccelerationVector, endPoint.getFramePointCopy(), zeroAccelerationVector);
      apexChangedSinceLastRedraw.set(true);
      updateBagOfBallsGraphics();
   }

   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      throw new RuntimeException("Do not use computeNextTick!");
   }

   private void updateBagOfBallsGraphics()
   {
      if (apexChangedSinceLastRedraw())
      {
         listOfBallPositions.clear();
         FramePoint tempPoint = new FramePoint(worldFrame, 0, 0, 0);

         for (int i = 0; i < numberOfStepsToVisualize; i++)
         {
            compute((getFinalTime() / numberOfStepsToVisualize) * i);
            packPosition(tempPoint);
            packVelocity(zeroAccelerationVector);
            packAcceleration(zeroAccelerationVector);
            AppearanceDefinition appearance;

            if (adjustmentMode)
               appearance = YoAppearance.Red();
            else
               appearance = YoAppearance.Black();

            if (i == numberOfStepsToVisualize - 1)
               appearance = YoAppearance.Yellow();

            bagOfBalls.setBallLoop(tempPoint, appearance);
            listOfBallPositions.add(new FramePoint(tempPoint));
         }
      }

      apexChangedSinceLastRedraw.set(false);
   }

   private boolean apexChangedSinceLastRedraw()
   {
      return apexChangedSinceLastRedraw.getBooleanValue();
   }

//   public void clicked3DPoint(MouseEvent mouseEvent, Point3d pointClicked, Point3d cameraPosition, Point3d fixPosition)
//   {
//      if (adjustmentMode)
//      {
//         adjustmentMode = false;
//         setApex(getCurrentGroundClearance());
//         updateBagOfBallsGraphics();
//         return;
//      }
//
//      boolean wasBallClicked = false;
//      System.out.println("Click! X: " + pointClicked.getX() + " Y: " + pointClicked.getY() + " Z: " + pointClicked.getZ());
//      for (FramePoint p : listOfBallPositions)
//      {
//         if (MathTools.epsilonEquals(p.getX(), pointClicked.getX(), 1e-2) && MathTools.epsilonEquals(p.getY(), pointClicked.getY(), 1e-2)
//               && MathTools.epsilonEquals(p.getZ(), pointClicked.getZ(), 1e-2))
//         {
//            wasBallClicked = true;
//         }
//      }
//
//      if (wasBallClicked)
//      {
//         adjustmentMode = true;
//         setApex(getCurrentGroundClearance());
//         updateBagOfBallsGraphics();
//      }
//   }

   public FramePoint getEndPoint()
   {
      return endPoint.getFramePointCopy();
   }
}
