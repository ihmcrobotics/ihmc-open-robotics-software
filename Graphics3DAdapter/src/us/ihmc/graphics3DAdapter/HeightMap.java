package us.ihmc.graphics3DAdapter;

public interface HeightMap
{
   public abstract double heightAt(double x, double y, double z);
   public abstract double getXMin();
   public abstract double getXMax();
   
   public abstract double getYMin();
   public abstract double getYMax();
    
}
