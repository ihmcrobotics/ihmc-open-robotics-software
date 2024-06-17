package us.ihmc.perception.YOLOv8;

public record YOLOv8SimpleDetection(YOLOv8DetectionClass objectClass,
                                    float confidence,
                                    int x, int y,
                                    int width, int height,
                                    float[] maskWeights)
{ }