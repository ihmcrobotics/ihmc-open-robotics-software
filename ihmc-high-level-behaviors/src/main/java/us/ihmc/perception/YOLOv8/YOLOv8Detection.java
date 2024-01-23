package us.ihmc.perception.YOLOv8;

public record YOLOv8Detection(int classId, String className,
                              float confidence,
                              int x, int y,
                              int width, int height,
                              float[] maskWeights)
{ }
