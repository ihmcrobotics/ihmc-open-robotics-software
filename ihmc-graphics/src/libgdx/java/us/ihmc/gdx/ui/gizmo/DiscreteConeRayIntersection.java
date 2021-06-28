package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class DiscreteConeRayIntersection
{
   public void intersect()
   {
      // update cone
//      tempPolytopeTransform.setToZero();
//      if (side == RobotSide.LEFT) // only show the cones in the positive direction
//      {
//         tempPolytopeTransform.getTranslation().addZ(0.5 * arrowSpacing + arrowBodyLength);
//         tempTransform.transform(tempPolytopeTransform);
//
//         arrowHeadCollisionSphere.setToZero();
//         double arrowSurroundingSphereRadius;
//         if (arrowHeadRadius > (0.5 * arrowHeadLength))
//            arrowSurroundingSphereRadius = arrowHeadRadius / Math.sin(Math.atan(2.0 * arrowHeadRadius / arrowHeadLength));
//         else
//            arrowSurroundingSphereRadius = 0.5 * arrowHeadLength;
//         arrowHeadCollisionSphere.setRadius(arrowSurroundingSphereRadius);
//         arrowHeadCollisionSphere.getPosition().addZ(0.5 * arrowSpacing + arrowBodyLength + 0.5 * arrowHeadLength);
//         arrowHeadCollisionSphere.applyTransform(tempTransform);
//
//         adjustedRayOrigin.setX(pickRay.getPoint().getX() - arrowHeadCollisionSphere.getPosition().getX());
//         adjustedRayOrigin.setY(pickRay.getPoint().getY() - arrowHeadCollisionSphere.getPosition().getY());
//         adjustedRayOrigin.setZ(pickRay.getPoint().getZ() - arrowHeadCollisionSphere.getPosition().getZ());
//         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(arrowHeadCollisionSphere.getRadius(),
//                                                                                                arrowHeadCollisionSphere.getRadius(),
//                                                                                                arrowHeadCollisionSphere.getRadius(),
//                                                                                                adjustedRayOrigin,
//                                                                                                pickRay.getDirection(),
//                                                                                                firstIntersectionToPack,
//                                                                                                secondIntersectionToPack);
//         if (numberOfIntersections == 2)
//         {
//            firstIntersectionToPack.add(arrowHeadCollisionSphere.getPosition());
//            secondIntersectionToPack.add(arrowHeadCollisionSphere.getPosition());
//
//            arrowHeadCollisionBaseFacingTip.setToZero();
//            arrowHeadCollisionBaseFacingTip.applyTransform(tempPolytopeTransform);
//
//            arrowHeadCollisionTipFacingBase.setToZero();
//            arrowHeadCollisionTipFacingBase.getPoint().addZ(arrowHeadLength);
//            arrowHeadCollisionTipFacingBase.getNormal().set(0.0, 0.0, -1.0);
//            arrowHeadCollisionTipFacingBase.applyTransform(tempPolytopeTransform);
//
//            arrowHeadCollisionAxis.set(arrowHeadCollisionBaseFacingTip.getPoint(), arrowHeadCollisionBaseFacingTip.getNormal());
//
//            for (int i = 0; i < 100; i++)
//            {
//               interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / 100.0);
//
//               if (arrowHeadCollisionBaseFacingTip.isOnOrAbove(interpolatedPoint) && arrowHeadCollisionTipFacingBase.isOnOrAbove(interpolatedPoint))
//               {
//                  arrowHeadCollisionAxis.orthogonalProjection(interpolatedPoint, arrowHeadCollisionAxisPoint);
//                  double distanceFromBase = arrowHeadCollisionAxisPoint.distance(arrowHeadCollisionBaseFacingTip.getPoint());
//                  double radiusBoundsAtTier = EuclidCoreTools.interpolate(arrowHeadRadius, 0.0, distanceFromBase / arrowHeadLength);
//                  if (arrowHeadCollisionAxisPoint.distance(interpolatedPoint) <= radiusBoundsAtTier)
//                  {
//                     double distance = interpolatedPoint.distance(pickRay.getPoint());
//                     if (distance < closestCollisionDistance)
//                     {
//                        closestCollisionDistance = distance;
//                        closestCollisionSelection = SixDoFSelection.toLinearSelection(axis);
//                        closestCollision.set(interpolatedPoint);
//                     }
//                     break;
//                  }
//               }
//            }
//         }
   }
}
