package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class AtlasFootstepSnappingParameters extends FootstepSnappingParameters
{
   public AtlasFootstepSnappingParameters()
   {
      super(null, //collision polygon, set later
            null, //support polygon, set later
            0.3, //bounding square length
            Math.PI / 6.0, //max Angle
            0.0, //min area, changed below
            0.02, //0.01, //zDistance tolerance
            0.04, //adjustment distance
            Math.PI / 24, //adjustment angle
            3); // 1); //number of outliers allowed during plane fitting

      double actualFootWidth = 0.138;
      double actualFootLength = 0.26;
      double footWidthForControl = 0.11; //0.12; // 0.08;   //0.124887;
      double toeWidthForControl = 0.085; //0.095; // 0.07;   //0.05;   //
      double footLengthForControl = 0.22; //0.255;
      double footBackForControl = 0.09; // 0.06;   //0.082;    // 0.07;

      double footOffset = (footLengthForControl - 2 * footBackForControl) / 2.0;

      collisionPolygon = new ConvexPolygon2d();
      collisionPolygon.addVertex(actualFootLength / 2.0 + footOffset, actualFootWidth / 2.0);
      collisionPolygon.addVertex(actualFootLength / 2.0 + footOffset, -actualFootWidth / 2.0);
      collisionPolygon.addVertex(-actualFootLength / 2.0 + footOffset, -actualFootWidth / 2.0);
      collisionPolygon.addVertex(-actualFootLength / 2.0 + footOffset, actualFootWidth / 2.0);
      collisionPolygon.update();

      supportPolygon = new ConvexPolygon2d();
      supportPolygon.addVertex(footLengthForControl - footBackForControl, toeWidthForControl / 2.0);
      supportPolygon.addVertex(footLengthForControl - footBackForControl, -toeWidthForControl / 2.0);
      supportPolygon.addVertex(-footBackForControl, footWidthForControl / 2.0);
      supportPolygon.addVertex(-footBackForControl, -footWidthForControl / 2.0);
      supportPolygon.update();
   }

   @Override
   public double getMinArea()
   {
      return supportPolygon.getArea() * 0.25;
   }
}
