package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

/**
 * Created by agrabertilton on 1/20/15.
 */
public class SimpleFootstepValueFunction implements FootstepValueFunction
{
   private QuadTreeFootstepSnappingParameters parameters;
   private double slopeGain = -0.25; //more slope is bad
   private double areaGain = 100; //more area is good

   public SimpleFootstepValueFunction(QuadTreeFootstepSnappingParameters parameters)
   {
      this.parameters = parameters;
      updateFunction();
   }

   @Override
   public double getFootstepValue(FootstepDataMessage footstepData)
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.set(footstepData.getOrientation());
      Vector3D footstepNormal = new Vector3D();
      rotationMatrix.getColumn(2, footstepNormal);

      double offHorizontalAngle = Math.acos(footstepNormal.getZ());

      if (offHorizontalAngle > parameters.getMaxAngle() || footstepNormal.getZ() < 0)
         return Double.NEGATIVE_INFINITY;

      double value = slopeGain * offHorizontalAngle;
      if (footstepData.getPredictedContactPoints2d() == null || footstepData.getPredictedContactPoints2d().isEmpty())
         return Double.NEGATIVE_INFINITY;

      ConvexPolygon2D supportPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(HumanoidMessageTools.unpackPredictedContactPoints(footstepData)));
      supportPolygon.update();
      double inPlaneArea = supportPolygon.getArea();
      double horizonalArea = inPlaneArea * footstepNormal.getZ();

      if (horizonalArea < parameters.getMinArea())
         return Double.NEGATIVE_INFINITY;

      value += horizonalArea * areaGain;
      return value;
   }

   @Override
   public void updateFunction()
   {
   }
}
