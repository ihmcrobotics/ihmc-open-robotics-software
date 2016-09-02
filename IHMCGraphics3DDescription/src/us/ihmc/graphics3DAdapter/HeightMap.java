package us.ihmc.graphics3DAdapter;

import us.ihmc.robotics.geometry.BoundingBox3d;

public interface HeightMap
{
   public abstract double heightAt(double x, double y, double z);
   public abstract BoundingBox3d getBoundingBox();
}
