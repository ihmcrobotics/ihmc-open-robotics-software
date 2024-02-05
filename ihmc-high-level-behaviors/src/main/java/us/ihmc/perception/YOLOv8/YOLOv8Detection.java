package us.ihmc.perception.YOLOv8;

public record YOLOv8Detection(YOLOv8DetectableObject objectClass,
                              float confidence,
                              int x, int y,
                              int width, int height,
                              float[] maskWeights)
{ }