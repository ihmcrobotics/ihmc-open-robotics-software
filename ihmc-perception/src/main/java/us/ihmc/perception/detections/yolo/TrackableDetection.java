package us.ihmc.perception.detections.yolo;

public interface TrackableDetection
{
   String getObjectClass();

   double getConfidence();

   float getX1();
   float getY1();
   float getX2();
   float getY2();

   boolean has3D();
   float getCx();
   float getCy();
   float getCz();

   void setTrackId(int id);
   int getTrackId();
}