package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class FrictionBasedDoubleSupportForceDistributor implements DoubleSupportForceDistributor
{

   private final YoVariableRegistry registry = new YoVariableRegistry("VirtualSupportActuators");
   private final BipedSupportPolygons bipedSupportPolygons;

   private final DoubleYoVariable leftYawStrength = new DoubleYoVariable("leftYawStrength", registry);
   private final DoubleYoVariable rightYawStrength = new DoubleYoVariable("rightYawStrength", registry);
   private final SideDependentList<DoubleYoVariable> yawStrengths = new SideDependentList<DoubleYoVariable>(leftYawStrength, rightYawStrength);

   private final ReferenceFrame pelvisFrame;

   public FrictionBasedDoubleSupportForceDistributor(ReferenceFrame pelvisFrame, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.pelvisFrame = pelvisFrame;
      parentRegistry.addChild(registry);
   }

   private void calculateYawStrengths(SideDependentList<FramePoint2d> virtualToePoints, SideDependentList<Double> legStrengths)
   {
      if (virtualToePoints == null)
      {
         leftYawStrength.set(legStrengths.get(RobotSide.LEFT));
         rightYawStrength.set(legStrengths.get(RobotSide.RIGHT));
         return;
      }

      SideDependentList<Double> minDistance = new SideDependentList<Double>();

      for (RobotSide robotSide : RobotSide.values)
      {
         minDistance.set(robotSide, Double.POSITIVE_INFINITY);

         FramePoint2d vtp = virtualToePoints.get(robotSide);

         FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(robotSide);
         for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
         {
            FramePoint2d point = footPolygon.getFrameVertex(i);
            point.changeFrame(vtp.getReferenceFrame());

            double dist = point.distance(vtp);

            if (dist < minDistance.get(robotSide))
            {
               minDistance.set(robotSide, dist);
            }
         }
      }

      double leftLegWeight = minDistance.get(RobotSide.LEFT) * legStrengths.get(RobotSide.LEFT);
      double rightLegWeight = minDistance.get(RobotSide.RIGHT) * legStrengths.get(RobotSide.RIGHT);
      double totalWeight = leftLegWeight + rightLegWeight;

      if (totalWeight < 1e-12)
         totalWeight = 1e-12;

      leftYawStrength.set(leftLegWeight / totalWeight);
      rightYawStrength.set(rightLegWeight / totalWeight);
   }

   public void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesInPelvisFrameToPack,
         double zForceInPelvisFrameTotal, FrameVector torqueInPelvisFrameTotal, SideDependentList<Double> legStrengths,
         SideDependentList<FramePoint2d> virtualToePoints)
   {
      calculateYawStrengths(virtualToePoints, legStrengths);

      for (RobotSide robotSide : RobotSide.values)
      {
         double legStrength = legStrengths.get(robotSide);

         zForcesInPelvisFrameToPack.put(robotSide, zForceInPelvisFrameTotal * legStrength);

         FrameVector torque = new FrameVector(torqueInPelvisFrameTotal);
         torque.changeFrame(pelvisFrame);

         torque.setX(torque.getX() * legStrength);
         torque.setY(torque.getY() * legStrength);
         torque.setZ(torque.getZ() * yawStrengths.get(robotSide).getDoubleValue());

         torquesInPelvisFrameToPack.set(robotSide, torque);
      }
   }
}
