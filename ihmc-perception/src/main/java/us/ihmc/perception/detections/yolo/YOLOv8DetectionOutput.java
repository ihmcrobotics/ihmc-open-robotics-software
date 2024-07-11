package us.ihmc.perception.detections.yolo;

public record YOLOv8DetectionOutput(YOLOv8DetectionClass objectClass,
                                    float confidence,
                                    int x, int y,
                                    int width, int height,
                                    float[] maskWeights)
{ }