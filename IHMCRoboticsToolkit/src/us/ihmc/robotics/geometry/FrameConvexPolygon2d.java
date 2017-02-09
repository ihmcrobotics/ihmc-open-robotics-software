package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import org.apache.commons.math3.util.Pair;

import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>Title: FrameConvexPolygon2d </p>
 *
 * <p>Description: Describes a planar convex polygon attached to a {@link ReferenceFrame}.
 * The vertices of this polygon are clockwise and are all different.
 * </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author IHMC-Yobotics Biped Team
 * @version 1.0
 */
public class FrameConvexPolygon2d extends FrameGeometry2d<FrameConvexPolygon2d, ConvexPolygon2d>
{
   protected final ConvexPolygon2d convexPolygon;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   private final FramePoint tempPoint = new FramePoint();
   private final FramePoint2d tempPoint2d = new FramePoint2d();

   private final FramePoint2d temporaryCentroid = new FramePoint2d();


   /**
    * Creates an empty convex polygon attached to the world frame.
    */
   public FrameConvexPolygon2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates an empty convex polygon attached to a reference frame.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new ConvexPolygon2d());
      this.convexPolygon = this.getGeometryObject();
      update();
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using a list of Point2d, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: pointList.size() can be greater or equal to numberOfVertices.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, List<Point2d> vertices)
   {
      this(referenceFrame);
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, Point2d[] vertices)
   {
      this(referenceFrame);
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using an array of Point2d, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param vertices {@code double[N][>=2]} the array of points that is used to creates the vertices. Each row contains one point whereas the (at least) two columns contains the coordinates x and y.
    * The number of vertices of this polygon will be equal to the length of the point array.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, double[][] vertices)
   {
      this(referenceFrame);
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   /**
    * Creates an empty convex polygon attached to the reference frame of the frame vertex, adds N new vertices using a list of {@code List<FramePoint2d>}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code List<FramePoint2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: pointList.size() can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public FrameConvexPolygon2d(List<FramePoint2d> frameVertices)
   {
      this();
      setIncludingFrameAndUpdate(frameVertices);
   }

   /**
    * Creates an empty convex polygon attached to the reference frame of the frame vertex, adds N new vertices using a list of {@code FrameTuple2dArrayList<FramePoint2d>}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code FrameTuple2dArrayList<FramePoint2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: pointList.size() can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public FrameConvexPolygon2d(FrameTuple2dArrayList<FramePoint2d> frameVertices)
   {
      this();
      setIncludingFrameAndUpdate(frameVertices);
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using the vertices of another {code ConvexPolygon2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, ConvexPolygon2d otherPolygon)
   {
      super(referenceFrame, new ConvexPolygon2d());
      this.convexPolygon = this.getGeometryObject();

      setIncludingFrameAndUpdate(referenceFrame, otherPolygon);
   }

   /**
    * Creates a polygon with the same properties as the other polygon.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon.
    */
   public FrameConvexPolygon2d(FrameConvexPolygon2d otherPolygon)
   {
      this();
      setIncludingFrameAndUpdate(otherPolygon);
   }

   /**
    * Creates a new convex polygon by combining two other convex polygons. The result is the smallest convex hull that contains both polygons.
    * Then it updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    *
    * @param firstPolygon {@code ConvexPolygon2d}
    * @param secondPolygon {@code ConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public FrameConvexPolygon2d(FrameConvexPolygon2d firstPolygon, FrameConvexPolygon2d secondPolygon)
   {
      this();
      setIncludingFrameAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * Note that calling that method doesn't generate garbage.
    */
   public void clear()
   {
      convexPolygon.clear();
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid, and is attached to a new reference frame.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * Note that calling that method doesn't generate garbage.
    */
   public void clear(ReferenceFrame referenceFrame)
   {
      convexPolygon.clear();
      this.referenceFrame = referenceFrame;
   }

   public void clearAndUpdate(ReferenceFrame referenceFrame)
   {
      convexPolygon.clearAndUpdate();
      this.referenceFrame = referenceFrame;
   }

   /**
    * Add a vertex to this polygon.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    * @throws ReferenceFrameMismatchException
    */
   public void addVertex(FramePoint2d vertex)
   {
      vertex.checkReferenceFrameMatch(referenceFrame);
      addVertex(vertex.getPoint());
   }

   public void addVertex(Point2d vertex)
   {
      convexPolygon.addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon.
    * Note that this method recycles memory.
    * @param x {@code double} first coordinate of the new vertex.
    * @param y {@code double} second coordinate of the new vertex.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertex(ReferenceFrame referenceFrame, double x, double y)
   {
      this.referenceFrame.checkReferenceFrameMatch(referenceFrame);
      convexPolygon.addVertex(x, y);
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex.changeFrame(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    */
   public void addVertexAndChangeFrame(FramePoint2d newVertex)
   {
      tempPoint2d.setIncludingFrame(newVertex);
      tempPoint2d.changeFrame(referenceFrame);
      convexPolygon.addVertex(tempPoint2d.getPoint());
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex.changeFrameAndProjectToXYPlane(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    */
   public void addVertexChangeFrameAndProjectToXYPlane(FramePoint2d newVertex)
   {
      tempPoint2d.setIncludingFrame(newVertex);
      tempPoint2d.changeFrameAndProjectToXYPlane(referenceFrame);
      convexPolygon.addVertex(tempPoint2d.getPoint());
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex2d.changeFrameAndProjectToXYPlane(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint} the new vertex (it is not modified).
    */
   public void addVertexByProjectionOntoXYPlane(FramePoint newVertex)
   {
      tempPoint.setIncludingFrame(newVertex);
      tempPoint.changeFrame(referenceFrame);
      addVertex(referenceFrame, tempPoint.getX(), tempPoint.getY());
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code FramePoint2d}.
    * Note that this method recycles memory.
    * @param vertices {@code List<FramePoint2d>} the list of new vertices (it is not modified).
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(List<FramePoint2d> vertices, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint2d vertex = vertices.get(i);
         addVertex(vertex);
      }
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code FramePoint2d}.
    * Note that this method recycles memory.
    * @param vertices {@code FrameTuple2dArrayList<FramePoint2d>} the list of new vertices (it is not modified).
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(FrameTuple2dArrayList<FramePoint2d> vertices, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint2d vertex = vertices.get(i);
         addVertex(vertex);
      }
   }

   public void addVertices(FramePoint2d[] vertices)
   {
      for (int i = 0; i < vertices.length; i++)
      {
         FramePoint2d vertex = vertices[i];
         addVertex(vertex);
      }
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code FramePoint2d} after having changed their frame projected them on this polygon frame.
    * Note that this method recycles memory.
    * @param vertices {@code List<FramePoint2d>} the list of new vertices (it is not modified).
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    */
   public void addVerticesByProjectionOntoXYPlane(List<FramePoint> vertices, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint vertex = vertices.get(i);
         addVertexByProjectionOntoXYPlane(vertex);
      }
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    * Note that this method recycles memory.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon that is used to add new vertices to this polygon.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(ConvexPolygon2d otherPolygon)
   {
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = otherPolygon.getVertex(i);
         addVertex(vertex);
      }
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    * Note that this method recycles memory.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon that is used to add new vertices to this polygon.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(FrameConvexPolygon2d otherPolygon)
   {
      referenceFrame.checkReferenceFrameMatch(otherPolygon.getReferenceFrame());

      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = otherPolygon.convexPolygon.getVertex(i);
         convexPolygon.addVertex(vertex);
      }
   }

   public void removeVertex(int indexOfVertexToRemove)
   {
      convexPolygon.removeVertex(indexOfVertexToRemove);
   }

   /**
    * Calls {@code ConvexPolygon2d.update()} and updates the {@code FramePoint}s of this {@code FrameConvecPolygon2d}.
    */
   public void update()
   {
      convexPolygon.update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code List<FramePoint2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(List<FramePoint2d> vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertexByProjectionOntoXYPlane(vertices.get(i)};
    * 3- {@code update()}.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    */
   public void setAndUpdate(List<FramePoint> vertices)
   {
      clear();
      for (int i = 0; i < vertices.size(); i++)
      {
         addVertexByProjectionOntoXYPlane(vertices.get(i));
      }
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertexByProjectionOntoXYPlane(vertices.get(i)};
    * 3- {@code update()}.
    * @param vertices {@code FramePoint[]} the array of points that is used to creates the vertices.
    */
   public void setAndUpdate(FramePoint[] vertices)
   {
      clear();
      for (int i = 0; i < vertices.length; i++)
      {
         addVertexByProjectionOntoXYPlane(vertices[i]);
      }
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertex(vertices.get(i)};
    * 3- {@code update()}.
    * @param vertices {@code FramePoint2d[]} the array of points that is used to creates the vertices.
    */
   public void setAndUpdate(FramePoint2d[] vertices)
   {
      clear();
      for (int i = 0; i < vertices.length; i++)
      {
         addVertex(vertices[i]);
      }
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(ConvexPolygon2d otherPolygon)
   {
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(FrameConvexPolygon2d otherPolygon)
   {
      referenceFrame.checkReferenceFrameMatch(otherPolygon.getReferenceFrame());
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(firstPolygon)};
    * 2- {@code addVertices(secondPolygon)};
    * 3- {@code update()}.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    * @param firstPolygon {@code FrameConvexPolygon2d}
    * @param secondPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(FrameConvexPolygon2d firstPolygon, FrameConvexPolygon2d secondPolygon)
   {
      referenceFrame.checkReferenceFrameMatch(firstPolygon.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(secondPolygon.getReferenceFrame());

      clear();
      addVertices(firstPolygon);
      addVertices(secondPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    */
   public void setByProjectionOntoXYPlaneAndUpdate(List<FramePoint> vertices, int numberOfVertices)
   {
      clear();
      addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(otherPolygon.getReferenceFrame())};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code FrameConvexPolygon2d}
    */
   public void setIncludingFrameAndUpdate(FrameConvexPolygon2d otherPolygon)
   {
      clear(otherPolygon.getReferenceFrame());
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(firstPolygon.getReferenceFrame())};
    * 2- {@code addVertices(firstPolygon)};
    * 2- {@code addVertices(secondPolygon)};
    * 3- {@code update()}.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    * @param firstPolygon {@code FrameConvexPolygon2d}
    * @param secondPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameAndUpdate(FrameConvexPolygon2d firstPolygon, FrameConvexPolygon2d secondPolygon)
   {
      referenceFrame = firstPolygon.getReferenceFrame();
      setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * This method does:
    * 1- {@code clear(referenceFrame)};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param otherPolygon {@code ConvexPolygon2d}
    */
   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, ConvexPolygon2d otherPolygon)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(referenceFrame)};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    */
   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, List<Point2d> vertices)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(vertices, vertices.size());
      update();
   }

   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, Point2d[] vertices)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(vertices, vertices.length);
      update();
   }

   /**
    * If the list of vertices is empty, this polygon will be empty and its reference frame won't be changed.
    * If the list of vertices is not empty, this method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<FramePoint2d>} the list of points that is used to creates the vertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameAndUpdate(List<FramePoint2d> vertices)
   {
      int numberOfVertices = vertices.size();
      if (numberOfVertices < 1 )
      {
         clear();
         update();
      }
      else
      {
         clear(vertices.get(0).getReferenceFrame());
         addVertices(vertices, numberOfVertices);
         update();
      }
   }

   /**
    * If the list of vertices is empty, this polygon will be empty and its reference frame won't be changed.
    * If the list of vertices is not empty, this method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code FrameTuple2dArrayList<FramePoint2d>} the list of points that is used to creates the vertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameAndUpdate(FrameTuple2dArrayList<FramePoint2d> vertices)
   {
      int numberOfVertices = vertices.size();
      if (numberOfVertices < 1 )
      {
         clear();
         update();
      }
      else
      {
         clear(vertices.get(0).getReferenceFrame());
         addVertices(vertices, numberOfVertices);
         update();
      }
   }

   /**
    * This method does:
    * 1- {@code clear(referenceFrame)};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code double[>=numberOfVertices][>=2]} the array of points that is used to creates the vertices.
    */
   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, double[][] vertices)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(vertices, vertices.length);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVerticesByProjectionOntoXYPlane(vertices, vertices.size())};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameByProjectionOntoXYPlaneAndUpdate(ReferenceFrame referenceFrame, List<FramePoint> vertices)
   {
      int numberOfVertices = vertices.size();
      clear(referenceFrame);
      addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int}.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameByProjectionOntoXYPlaneAndUpdate(ReferenceFrame referenceFrame, List<FramePoint> vertices, int numberOfVertices)
   {
      clear(referenceFrame);
      addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices);
      update();
   }

   public void getVertex(int vertexIndex, Point2d vertexToPack)
   {
      vertexToPack.set(getVertex(vertexIndex));
   }

   public Point2d getVertex(int vertexIndex)
   {
      return convexPolygon.getVertex(vertexIndex);
   }

   public FramePoint2d getFrameVertexCopy(int vertexIndex)
   {
      FramePoint2d frameVertexCopy = new FramePoint2d();
      getFrameVertex(vertexIndex, frameVertexCopy);
      return frameVertexCopy;
   }

   public void getFrameVertex(int vertexIndex, FrameTuple2d<?, ?> vertexToPack)
   {
      convexPolygon.checkIfUpToDate();

      convexPolygon.checkNonEmpty();
      convexPolygon.checkIndexInBoundaries(vertexIndex);

      vertexToPack.setIncludingFrame(referenceFrame, convexPolygon.getVertex(vertexIndex));
   }

   public void getFrameVertexXY(int vertexIndex, FramePoint vertexToPack)
   {
      convexPolygon.checkIfUpToDate();

      convexPolygon.checkNonEmpty();
      convexPolygon.checkIndexInBoundaries(vertexIndex);

      vertexToPack.setXYIncludingFrame(referenceFrame, convexPolygon.getVertex(vertexIndex));
   }

   public void getNextFrameVertex(int vertexIndex, FramePoint2d vertexToPack)
   {
      getFrameVertex(convexPolygon.getNextVertexIndex(vertexIndex), vertexToPack);
   }

   public void getPreviousFrameVertex(int vertexIndex, FramePoint2d vertexToPack)
   {
      getFrameVertex(convexPolygon.getPreviousVertexIndex(vertexIndex), vertexToPack);
   }

   public double getArea()
   {
      return convexPolygon.getArea();
   }

   public void getCentroid(FramePoint2d centroidToPack)
   {
      centroidToPack.setIncludingFrame(referenceFrame, convexPolygon.getCentroid());
   }

   public FramePoint2d getCentroid()
   {
      getCentroid(temporaryCentroid);
      return temporaryCentroid;
   }

   public FramePoint2d getCentroidCopy()
   {
      FramePoint2d centroidToReturn = new FramePoint2d();
      getCentroid(centroidToReturn);
      return centroidToReturn;
   }

   public int getNumberOfVertices()
   {
      return convexPolygon.getNumberOfVertices();
   }

   public ConvexPolygon2d getConvexPolygon2d()
   {
      convexPolygon.checkIfUpToDate();
      return convexPolygon;
   }

   public ConvexPolygon2d getConvexPolygon2dCopy()
   {
      convexPolygon.checkIfUpToDate();
      return new ConvexPolygon2d(convexPolygon);
   }

   /**
    * Scale this convex polygon about pointToScaleAbout.
    * @param pointToScaleAbout
    * @param scaleFactor
    */
   public void scale(Point2d pointToScaleAbout, double scaleFactor)
   {
      convexPolygon.scale(pointToScaleAbout, scaleFactor);
   }

   /**
    * Scale this convex polygon about pointToScaleAbout.
    * @param pointToScaleAbout
    * @param scaleFactor
    */
   public void scale(FramePoint2d pointToScaleAbout, double scaleFactor)
   {
      checkReferenceFrameMatch(pointToScaleAbout);
      scale(pointToScaleAbout.getPoint(), scaleFactor);
   }

   /**
    * Scale this convex polygon about its centroid, i.e. once scaled the polygon centroid remains unchanged.
    * @param scaleFactor
    */
   public void scale(double scaleFactor)
   {
      convexPolygon.scale(scaleFactor);
   }

   /**
    * Returns distance from the point to the boundary of this polygon. The return value
    * is positive if the point is inside and negative if it is outside.
    */
   public double getSignedDistance(FramePoint2d point)
   {
      return ConvexPolygon2dCalculator.getSignedDistance(point.tuple, this.convexPolygon);
   }

   public BoundingBox2d getBoundingBoxCopy()
   {
      BoundingBox2d ret = this.convexPolygon.getBoundingBoxCopy();

      return ret;
   }

   public void getBoundingBox(BoundingBox2d boundingBoxToPack)
   {
      this.convexPolygon.getBoundingBox(boundingBoxToPack);
   }

   public boolean isPointInside(FramePoint2d framePoint)
   {
      return isPointInside(framePoint, 0.0);
   }

   /**
    * isPointInside
    * Determines whether a point is inside the convex polygon (point in polygon test).
    * Uses the orientation method (Nordbeck, Rystedt, 1967)
    * Test is only valid for convex polygons, and only if the vertices are ordered clockwise.
    *
    * @param framePoint FramePoint2d the point to be tested
    * @return boolean true if the point is inside the polygon
    */
   public boolean isPointInside(FramePoint2d framePoint, double epsilon)
   {
      framePoint.checkReferenceFrameMatch(referenceFrame);
      return ConvexPolygon2dCalculator.isPointInside(framePoint.tuple, epsilon, this.convexPolygon);
   }

   /**
    * Returns all of the vertices that are visible from the observerPoint2d, in left to right order.
    * If the observerPoint2d is inside the polygon, returns null.
    *
    * @param observerFramePoint Point2d
    * @return Point2d[]
    */
   public ArrayList<FramePoint2d> getAllVisibleVerticesFromOutsideLeftToRightCopy(FramePoint2d observerFramePoint)
   {
      this.checkReferenceFrameMatch(observerFramePoint);
      int[] lineOfSightIndeces = ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observerFramePoint.getPoint(), convexPolygon);
      if (lineOfSightIndeces == null)
         return null;

      ArrayList<FramePoint2d> ret = new ArrayList<FramePoint2d>();
      int index = lineOfSightIndeces[0];

      while (true)
      {
         ret.add(getFrameVertexCopy(index));
         index = convexPolygon.getPreviousVertexIndex(index);
         if (index == lineOfSightIndeces[1])
         {
            ret.add(getFrameVertexCopy(index));
            break;
         }
      }

      return ret;
   }

   /**
    * Returns the intersecting edges between this polygon and the given FrameLine2d.
    * The FrameLine2d is treated as a ray. This method returns null if:
    * - The start point of the ray starts inside the Polygon,
    * - The ray points away from the polygon.
    * - The intersection is a single vertex,
    * - There is no intersection.
    *
    * @param frameLine2d FrameLine2d Ray to check intersection with this Polygon.
    * @return FrameLineSegment2d[] The two edges that the Ray intersects on this Polygon.
    */
   public FrameLineSegment2d[] getIntersectingEdges(FrameLine2d frameLine2d)
   {
      frameLine2d.checkReferenceFrameMatch(referenceFrame);

      LineSegment2d[] lineSegments = ConvexPolygon2dCalculator.getIntersectingEdgesCopy(frameLine2d.getLine2dCopy(), convexPolygon);
      if (lineSegments == null)
         return null;

      FrameLineSegment2d[] ret = new FrameLineSegment2d[lineSegments.length];

      for (int i = 0; i < lineSegments.length; i++)
      {
         ret[i] = new FrameLineSegment2d(referenceFrame, lineSegments[i]);
      }

      return ret;
   }

   public FramePoint2d getClosestVertexCopy(FramePoint2d point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      return new FramePoint2d(referenceFrame, ConvexPolygon2dCalculator.getClosestVertexCopy(point.getPoint(), convexPolygon));
   }

   public boolean getClosestPointWithRay(FramePoint2d closestVertexToPack, FrameLine2d ray)
   {
      ray.checkReferenceFrameMatch(referenceFrame);
      closestVertexToPack.setToZero(referenceFrame);
      boolean success = convexPolygon.getClosestPointWithRay(closestVertexToPack.getPoint(), ray.getLine2d());

      return success;
   }

   public void getClosestVertex(FramePoint2d closestVertexToPack, FramePoint2d point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      closestVertexToPack.setIncludingFrame(referenceFrame, ConvexPolygon2dCalculator.getClosestVertexCopy(point.getPoint(), convexPolygon));
   }

   public FramePoint2d getClosestVertexCopy(FrameLine2d line)
   {
      line.checkReferenceFrameMatch(referenceFrame);

      Point2d closestVertexCopy = ConvexPolygon2dCalculator.getClosestVertexCopy(line.line, convexPolygon);

      if (closestVertexCopy == null)
         throw new RuntimeException("Closest vertex could not be found! Has at least one vertex: " + convexPolygon.hasAtLeastOneVertex());

      return new FramePoint2d(referenceFrame, closestVertexCopy);
   }

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      convexPolygon.applyTransformAndProjectToXYPlane(transform);
   }

   @Override
   public FrameConvexPolygon2d applyTransformCopy(RigidBodyTransform transform)
   {
      FrameConvexPolygon2d copy = new FrameConvexPolygon2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public FrameConvexPolygon2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform)
   {
      FrameConvexPolygon2d copy = new FrameConvexPolygon2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   @Override
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      referenceFrame = desiredFrame;
      applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
   }

   @Override
   public FrameConvexPolygon2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FrameConvexPolygon2d ret = new FrameConvexPolygon2d(this);
      ret.changeFrameAndProjectToXYPlane(desiredFrame);
      return ret;
   }

   @Override
   public String toString()
   {
      FramePoint2d vertex = new FramePoint2d();

      String ret = "";
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         this.getFrameVertex(i, vertex);
         ret = ret + vertex.toString() + "\n";
      }

      return ret;
   }

   @Override
   public void orthogonalProjection(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      convexPolygon.orthogonalProjection(point.getPoint());
   }

   @Override
   public FramePoint2d orthogonalProjectionCopy(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      Point2d projected = convexPolygon.orthogonalProjectionCopy(point.getPoint());
      if (projected == null)
      {
         return null;
      }

      return new FramePoint2d(point.getReferenceFrame(), projected);
   }

   public void getNormal3dVector(FrameVector normalToPack)
   {
      normalToPack.setIncludingFrame(getReferenceFrame(), 0.0, 0.0, 1.0);
   }
   
   public void getPlane3d(FramePlane3d plane3dToPack)
   {
      plane3dToPack.setIncludingFrame(getReferenceFrame(), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
   }
   
   /**
    * @deprecated Creates garbage. Use an instance of FrameConvexPolygonWithLineIntersector.
    */
   @Override
   public FramePoint2d[] intersectionWith(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      Point2d[] intersection = this.convexPolygon.intersectionWith(line.line);
      if (intersection == null)
         return null;

      FramePoint2d[] ret = new FramePoint2d[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2d(line.referenceFrame, intersection[i]);
      }

      return ret;
   }
   
   public void intersectionWith(FrameLine2d otherLine, Pair<FramePoint2d, FramePoint2d> intersection)
   {
      checkReferenceFrameMatch(otherLine);
      int numberOfIntersections = ConvexPolygon2dCalculator.intersectionWithLine(otherLine.getLine2d(), intersection.getFirst().getPoint(),
                                                                                 intersection.getSecond().getPoint(), convexPolygon);
      if (numberOfIntersections < 2)
      {
         intersection.getSecond().setToNaN();
         if (numberOfIntersections < 1)
            intersection.getFirst().setToNaN();
      }
   }

   /**
    * Computes the intersections of a ray with this polygon. Since the polygon is convex the maximum
    * number of intersections is two. Returns the number of intersections found. If there are less
    * then two intersections the FramePoints are set to NaN.
    *
    * @param ray                  ray to intersect this polygon with
    * @param intersectionToPack1  modified - is set to the first intersection
    *                             If the are no intersections this will be set to NaN.
    * @param intersectionToPack2  modified - is set to the second intersection
    *                             If there is only one intersection this will be set to NaN.
    * @return                     The number of intersections 0, 1, or 2
    */
   public int intersectionWithRay(FrameLine2d ray, FramePoint2d intersectionToPack1, FramePoint2d intersectionToPack2)
   {
      checkReferenceFrameMatch(ray);
      intersectionToPack1.setToZero(referenceFrame);
      intersectionToPack2.setToZero(referenceFrame);
      return convexPolygon.intersectionWithRay(ray.getLine2d(), intersectionToPack1.getPoint(), intersectionToPack2.getPoint());
   }

   public FramePoint2d[] intersectionWithRayCopy(FrameLine2d ray)
   {
      checkReferenceFrameMatch(ray);
      Point2d[] intersections = convexPolygon.intersectionWithRayCopy(ray.getLine2dCopy());
      if (intersections == null)
         return null;

      FramePoint2d[] ret = new FramePoint2d[intersections.length];
      for (int i = 0; i < intersections.length; i++)
      {
         ret[i] = new FramePoint2d(referenceFrame, intersections[i]);
      }

      return ret;
   }

   @Override
   public FramePoint2d[] intersectionWith(FrameLineSegment2d lineSegment)
   {
      //TODO: Memory inefficient. Don't create new objects...
      checkReferenceFrameMatch(lineSegment);
      Point2d[] intersection = this.convexPolygon.intersectionWith(lineSegment.lineSegment);
      if (intersection == null)
         return null;

      FramePoint2d[] ret = new FramePoint2d[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2d(lineSegment.referenceFrame, intersection[i]);
      }

      return ret;
   }

   @Override
   public FrameConvexPolygon2d intersectionWith(FrameConvexPolygon2d secondConvexPolygon)
   {
      checkReferenceFrameMatch(secondConvexPolygon);
      ConvexPolygon2d intersection = this.convexPolygon.intersectionWith(secondConvexPolygon.convexPolygon);
      if (intersection == null)
         return null;

      return new FrameConvexPolygon2d(secondConvexPolygon.getReferenceFrame(), intersection);
   }

   public boolean intersectionWith(FrameConvexPolygon2d secondConvexPolygon, FrameConvexPolygon2d intersectionToPack)
   {
      checkReferenceFrameMatch(secondConvexPolygon);
      intersectionToPack.clear(secondConvexPolygon.getReferenceFrame());
      boolean success = convexPolygon.intersectionWith(secondConvexPolygon.convexPolygon, intersectionToPack.convexPolygon);

      return success;
   }

   @Override
   public double distance(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.convexPolygon.distance(point.getPoint());
   }

   public FrameLineSegment2d getClosestEdgeCopy(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return new FrameLineSegment2d(referenceFrame, convexPolygon.getClosestEdgeCopy(point.getPoint()));
   }

   public void getClosestEdge(FrameLineSegment2d closestEdgeToPack, FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      closestEdgeToPack.referenceFrame = referenceFrame;
      convexPolygon.getClosestEdge(closestEdgeToPack.lineSegment, point.getPoint());
   }

   @Override
   public double distance(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      return convexPolygon.distance(line.line);
   }

   @Override
   public double distance(FrameLineSegment2d lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      return convexPolygon.distance(lineSegment.lineSegment);
   }

   @Override
   public double distance(FrameConvexPolygon2d secondConvexPolygon)
   {
      checkReferenceFrameMatch(secondConvexPolygon);
      return convexPolygon.distance(secondConvexPolygon.convexPolygon);
   }

   public double getMaxX()
   {
      return convexPolygon.getMaxX();
   }

   public double getMinX()
   {
      return convexPolygon.getMinX();
   }

   public double getMaxY()
   {
      return convexPolygon.getMaxY();
   }

   public double getMinY()
   {
      return convexPolygon.getMinY();
   }

   public FramePoint2d getMinXMaxYPointCopy()
   {
      return new FramePoint2d(referenceFrame, convexPolygon.getMinXMaxYPointCopy());
   }

   public FramePoint2d getMinXMinYPointCopy()
   {
      return new FramePoint2d(referenceFrame, convexPolygon.getMinXMinYPointCopy());
   }

   public FramePoint2d getMaxXMaxYPointCopy()
   {
      return new FramePoint2d(referenceFrame, convexPolygon.getMaxXMaxYPointCopy());
   }

   public FramePoint2d getMaxXMinYPointCopy()
   {
      return new FramePoint2d(referenceFrame, convexPolygon.getMaxXMinYPointCopy());
   }

   public boolean isUpToDate()
   {
      return convexPolygon.isUpToDate();
   }

   public boolean isEmpty()
   {
      return convexPolygon.isEmpty();
   }

   @Override
   public void set(FrameConvexPolygon2d other)
   {
      convexPolygon.set(other.convexPolygon);
      update();
   }

   public boolean getLineOfSightVertexIndices(FramePoint2d observer, int[] idicesToPack)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2dCalculator.getLineOfSightVertexIndices(observer.getPoint(), idicesToPack, convexPolygon);
   }

   // --- methods that generate garbage ---
   public int[] getLineOfSightVertexIndicesCopy(FramePoint2d observer)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer.getPoint(), convexPolygon);
   }

   public FramePoint2d[] getLineOfSightVerticesCopy(FramePoint2d observer)
   {
      checkReferenceFrameMatch(observer);
      Point2d[] vertices = ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer.getPoint(), convexPolygon);
      FramePoint2d point1 = new FramePoint2d(getReferenceFrame(), vertices[0]);
      FramePoint2d point2 = new FramePoint2d(getReferenceFrame(), vertices[1]);
      return new FramePoint2d[] {point1, point2};
   }

}
