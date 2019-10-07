package us.ihmc.robotics.geometry;

// Colinear parallel edge intersection not possible due to EuclidGeometryTools#intersectionBetweenLine2DAndLineSegment2D implementation,
// which reads "When the line and the line segment are collinear, they are assumed to intersect at lineSegmentStart."
public enum ConvexPolygonCutResult
{
   KEEP_ALL,
   REMOVE_ALL,
   CUT
}
