package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.XYPlaneFrom3PointsFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import Jama.Matrix;

/**
 * This turns out to be exactly the same as TeeterTotterLegStrengthCalculator!
 */
public class ForceDecompositionLegStrengthCalculator implements LegStrengthCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("LegStrengthCalculator");

   private final ReferenceFrame midFeetZUpFrame;

   private final ReferenceFrame endEffectorFrame;
   private final XYPlaneFrom3PointsFrame vtpAndEndEffectorXYPlaneFrame;

   private final SideDependentList<FramePoint> vtps = new SideDependentList<FramePoint>();
   private final FramePoint desiredCoP;
   private final FramePoint endEffector;

   private final FrameVector zUnitVector;
   private final SideDependentList<FrameVector> vtpToEndEffectorUnitVectors = new SideDependentList<FrameVector>();
   private final FrameVector copToEndEffectorUnitVector;

   private final FrameVector fBar3D;

   private final Matrix E = new Matrix(2, 2);
   private final Matrix fBar = new Matrix(2, 1);

   private double epsilonInPlane = 1e-4;
   
   private final SideDependentList<DoubleYoVariable> legStrengths = new SideDependentList<DoubleYoVariable>();

   public ForceDecompositionLegStrengthCalculator(ReferenceFrame midFeetZUpFrame, ReferenceFrame endEffectorFrame, YoVariableRegistry parentRegistry)
   {
      this.midFeetZUpFrame = midFeetZUpFrame;
      this.endEffectorFrame = endEffectorFrame;
      this.vtpAndEndEffectorXYPlaneFrame = new XYPlaneFrom3PointsFrame(midFeetZUpFrame, "vtpAndEndEffectorXYPlaneFrame");

      for (RobotSide robotSide : RobotSide.values)
      {
         vtps.put(robotSide, new FramePoint(midFeetZUpFrame));
      }

      desiredCoP = new FramePoint(midFeetZUpFrame);
      endEffector = new FramePoint(endEffectorFrame);

      zUnitVector = new FrameVector(vtpAndEndEffectorXYPlaneFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         vtpToEndEffectorUnitVectors.put(robotSide, new FrameVector(vtpAndEndEffectorXYPlaneFrame));
      }

      copToEndEffectorUnitVector = new FrameVector(vtpAndEndEffectorXYPlaneFrame);
      fBar3D = new FrameVector(vtpAndEndEffectorXYPlaneFrame);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         legStrengths.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "LegStrength", registry));
      }
      parentRegistry.addChild(registry);
   }

   public void packLegStrengths(SideDependentList<Double> legStrengths, SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d coPDesired)
   {
      updatePoints(virtualToePoints, coPDesired);

      updateVTPAndEndEffectorXYPlaneFrame();

      updateVectors();

      for (RobotSide robotSide : RobotSide.values)
      {
         int column = robotSide.ordinal();
         
         FrameVector vtpToEndEffectorUnitVector = vtpToEndEffectorUnitVectors.get(robotSide);
         E.set(0, column, vtpToEndEffectorUnitVector.getX());
         E.set(1, column, vtpToEndEffectorUnitVector.getY());
      }

      fBar.set(0, 0, fBar3D.getX());
      fBar.set(1, 0, fBar3D.getY());

      Matrix c = E.solve(fBar);
      for (RobotSide robotSide : RobotSide.values)
      {
         int row = robotSide.ordinal();
         double legStrength = c.get(row, 0) * vtpToEndEffectorUnitVectors.get(robotSide).dot(zUnitVector);
         legStrengths.put(robotSide, legStrength);
      }

      doChecks(legStrengths);
      for (RobotSide robotSide : RobotSide.values)
      {
         this.legStrengths.get(robotSide).set(legStrengths.get(robotSide));
      }
   }

   private void doChecks(SideDependentList<Double> legStrengths)
   {
      double sum = MathTools.sumDoubles(legStrengths.values());
      if (!MathTools.epsilonEquals(sum, 1.0, 1e-2))
      {
         throw new RuntimeException("Leg strengths don't add up to 1");
      }
   }



   private void updatePoints(SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d coPDesired)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d vtp2d = virtualToePoints.get(robotSide);
         vtp2d.changeFrame(midFeetZUpFrame);
         vtps.get(robotSide).setIncludingFrame(vtp2d.getReferenceFrame(), vtp2d.getX(), vtp2d.getY(), 0.0);
      }

      coPDesired.changeFrame(midFeetZUpFrame);
      desiredCoP.setIncludingFrame(coPDesired.getReferenceFrame(), coPDesired.getX(), coPDesired.getY(), 0.0);

      endEffector.setIncludingFrame(endEffectorFrame, 0.0, 0.0, 0.0);
      endEffector.changeFrame(midFeetZUpFrame);
   }

   private void updateVTPAndEndEffectorXYPlaneFrame()
   {
      vtpAndEndEffectorXYPlaneFrame.setPoints(vtps.get(RobotSide.LEFT), vtps.get(RobotSide.RIGHT), endEffector);
      vtpAndEndEffectorXYPlaneFrame.update();
   }
   
   private void updateVectors()
   {
      zUnitVector.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
      zUnitVector.changeFrame(vtpAndEndEffectorXYPlaneFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint vtp = vtps.get(robotSide);
         vtp.changeFrame(vtpAndEndEffectorXYPlaneFrame);
         MathTools.checkIfInRange(vtp.getZ(), -epsilonInPlane, epsilonInPlane);
      }
      desiredCoP.changeFrame(vtpAndEndEffectorXYPlaneFrame);
//      MathTools.checkIfInRange(desiredCoP.getZ(), -epsilonInPlane, epsilonInPlane); // FIXME
      
      endEffector.changeFrame(vtpAndEndEffectorXYPlaneFrame);
      MathTools.checkIfInRange(endEffector.getZ(), -epsilonInPlane, epsilonInPlane);

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameVector vtpToEndEffectorUnitVector = vtpToEndEffectorUnitVectors.get(robotSide);
         vtpToEndEffectorUnitVector.sub(endEffector, vtps.get(robotSide));
         vtpToEndEffectorUnitVector.normalize();
      }

      copToEndEffectorUnitVector.sub(endEffector, desiredCoP);
      copToEndEffectorUnitVector.normalize();
      fBar3D.set(copToEndEffectorUnitVector);
      fBar3D.scale(1.0 / zUnitVector.dot(copToEndEffectorUnitVector));
   }
}
