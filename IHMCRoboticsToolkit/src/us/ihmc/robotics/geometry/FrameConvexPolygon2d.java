package us.ihmc.robotics.geometry;

import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import java.util.ArrayList;
import java.util.List;

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
public class FrameConvexPolygon2d extends FrameGeometry2d
{
   protected ReferenceFrame referenceFrame;
   private final FramePoint2d centroid = new FramePoint2d();
   protected final ConvexPolygon2d convexPolygon = new ConvexPolygon2d();
   private final List<FramePoint2d> frameVertices = new ArrayList<FramePoint2d>();
   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();
   
   private Vector2d[] temporaryVectorArray;

   private final FramePoint tempPoint = new FramePoint();

   /**
    * Creates an empty convex polygon attached to the world frame.
    */
   public FrameConvexPolygon2d()
   {
      referenceFrame = ReferenceFrame.getWorldFrame();
      update();
   }

   /**
    * Creates an empty convex polygon attached to a reference frame.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
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
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, Point2d[] vertices)
   {
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
      setIncludingFrameAndUpdate(frameVertices);
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using the vertices of another {code ConvexPolygon2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, ConvexPolygon2d otherPolygon)
   {
      setIncludingFrameAndUpdate(referenceFrame, otherPolygon);
   }

   /**
    * Creates a polygon with the same properties as the other polygon.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon.
    */
   public FrameConvexPolygon2d(FrameConvexPolygon2d otherPolygon)
   {
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
      updateFramePoints();
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
      setOrCreateIncludingFrame(vertex, getNumberOfVertices());
      convexPolygon.addVertex(vertex.getPoint());
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
      setOrCreateIncludingFrame(referenceFrame, x, y, getNumberOfVertices());
      convexPolygon.addVertex(x, y);
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex.changeFrame(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    */
   public void addVertexAndChangeFrame(FramePoint2d newVertex)
   {
      int vertexIndex = getNumberOfVertices();
      setOrCreateIncludingFrame(newVertex, vertexIndex);
      FramePoint2d vertex = frameVertices.get(vertexIndex);
      vertex.changeFrame(referenceFrame);
      convexPolygon.addVertex(vertex.getPoint());
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex.changeFrameAndProjectToXYPlane(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    */
   public void addVertexChangeFrameAndProjectToXYPlane(FramePoint2d newVertex)
   {
      int vertexIndex = getNumberOfVertices();
      setOrCreateIncludingFrame(newVertex, vertexIndex);
      FramePoint2d vertex = frameVertices.get(vertexIndex);
      vertex.changeFrameAndProjectToXYPlane(referenceFrame);
      convexPolygon.addVertex(vertex.getPoint());
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
         vertex.checkReferenceFrameMatch(referenceFrame);
         setOrCreateIncludingFrame(vertex, getNumberOfVertices());
         convexPolygon.addVertex(vertex.getPoint());
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
         vertex.checkReferenceFrameMatch(referenceFrame);
         setOrCreateIncludingFrame(vertex, getNumberOfVertices());
         convexPolygon.addVertex(vertex.getPoint());
      }
   }

   public void addVertices(FramePoint2d[] vertices)
   {
      for (int i = 0; i < vertices.length; i++)
      {
         FramePoint2d vertex = vertices[i];
         vertex.checkReferenceFrameMatch(referenceFrame);
         setOrCreateIncludingFrame(vertex, getNumberOfVertices());
         convexPolygon.addVertex(vertex.getPoint());
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
         setOrCreateIncludingFrame(vertex, referenceFrame, getNumberOfVertices());
         convexPolygon.addVertex(vertex);
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
         FramePoint2d vertex = otherPolygon.getFrameVertex(i);
         setOrCreateIncludingFrame(vertex, getNumberOfVertices());
         convexPolygon.addVertex(vertex.getPoint());
      }
   }

   public void removeVertex(int indexOfVertexToRemove)
   {
      convexPolygon.removeVertex(indexOfVertexToRemove);
      updateFramePoints();
   }

   /**
    * Calls {@code ConvexPolygon2d.update()} and updates the {@code FramePoint}s of this {@code FrameConvecPolygon2d}.
    */
   public void update()
   {
      convexPolygon.update();
      updateFramePoints();
   }
   
   /**
    * Updates the {@code FramePoint}s of this polygon.
    */
   public void updateFramePoints()
   {
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = convexPolygon.getVertexUnsafe(i);
         setOrCreateIncludingFrame(vertex, referenceFrame, i);
      }

      if (convexPolygon.isUpToDate())
         centroid.setIncludingFrame(referenceFrame, convexPolygon.getCentroid());
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

   public FramePoint2d getFrameVertex(int vertexIndex)
   {
      convexPolygon.checkIfUpToDate();
      return getFrameVertexUnsafe(vertexIndex);
   }

   /** Same as getVertex(vertexIndex) but without checking if the polygon has been updated. Be careful when using it! */
   protected FramePoint2d getFrameVertexUnsafe(int vertexIndex)
   {
      convexPolygon.checkNonEmpty();
      convexPolygon.checkIndexInBoundaries(vertexIndex);
      return frameVertices.get(vertexIndex);
   }

   public FramePoint2d getNextFrameVertex(int vertexIndex)
   {
      return getFrameVertex(convexPolygon.getNextVertexIndex(vertexIndex));
   }

   public FramePoint2d getPreviousFrameVertex(int vertexIndex)
   {
      return getFrameVertex(convexPolygon.getPreviousVertexIndex(vertexIndex));
   }

   private void setOrCreateIncludingFrame(FramePoint2d framePoint, int i)
   {
      setOrCreateIncludingFrame(framePoint.getReferenceFrame(), framePoint.getX(), framePoint.getY(), i);
   }

   private void setOrCreateIncludingFrame(Point2d point, ReferenceFrame referenceFrame, int i)
   {
      setOrCreateIncludingFrame(referenceFrame, point.getX(), point.getY(), i);
   }

   private void setOrCreateIncludingFrame(ReferenceFrame referenceFrame, double x, double y, int i)
   {
      while(i >= frameVertices.size())
         frameVertices.add(new FramePoint2d());
      frameVertices.get(i).setIncludingFrame(referenceFrame, x, y);
   }

   public double getArea()
   {
      return convexPolygon.getArea();
   }

   public void getCentroid(FramePoint2d centroidToPack)
   {
      convexPolygon.checkIfUpToDate();
      centroidToPack.setIncludingFrame(centroid);
   }

   public FramePoint2d getCentroid()
   {
      convexPolygon.checkIfUpToDate();
      return centroid;
   }

   public FramePoint2d getCentroidCopy()
   {
      convexPolygon.checkIfUpToDate();
      return new FramePoint2d(centroid);
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
      updateFramePoints();
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
      scale(centroid.getPoint(), scaleFactor);
   }

   /**
    * Returns distance from the point to the boundary of this polygon.
    * Positive number if inside. Negative number if outside.
    * If inside, the distance is the exact distance to an edge.
    * If outside, the negative distance is an underestimate.
    * It is actually the furthest distance from a projected edge that
    * it is outside of.
    * @param point
    * @return distance from point to this polygon.
    */
   public double getDistanceInside(FramePoint2d point)
   {
      return this.convexPolygon.getDistanceInside(point.tuple);
   }
   
   public BoundingBox2d getBoundingBoxCopy()
   {
      BoundingBox2d ret = this.convexPolygon.getBoundingBoxCopy();

      return ret;
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

      return convexPolygon.isPointInside(framePoint.tuple, epsilon);
   }

   /**
    * areAllPointsInside
    * Determines whether all the points in framePoints are inside the convex
    * polygon.
    *
    * @param framePoints FramePoint2d[]
    * @return boolean
    */
   public boolean areAllPointsInside(FramePoint2d[] framePoints)
   {
      ReferenceFrame referenceFrame = framePoints[0].getReferenceFrame();

      // Check and strip reference frames
      Point2d[] points = new Point2d[framePoints.length];
      for (int i = 0; i < framePoints.length; i++)
      {
         framePoints[i].checkReferenceFrameMatch(referenceFrame);
         points[i] = framePoints[i].getPointCopy();
      }

      return convexPolygon.areAllPointsInside(points);
   }

   public FrameLine2d[] getLinesOfSight(FramePoint2d observerFramePoint)
   {
      this.checkReferenceFrameMatch(observerFramePoint);
      Line2d[] linesOfSight = this.convexPolygon.getLinesOfSight(observerFramePoint.getPointCopy());

      if (linesOfSight == null)
         return null;

      FrameLine2d[] ret = new FrameLine2d[linesOfSight.length];
      for (int i = 0; i < linesOfSight.length; i++)
      {
         ret[i] = new FrameLine2d(observerFramePoint.getReferenceFrame(), linesOfSight[i]);
      }

      return ret;
   }

   /**
    * Returns all of the vertices that are visible from the observerPoint2d, in left to right order.
    * If the observerPoint2d is inside the polygon, returns null.
    *
    * @param observerFramePoint Point2d
    * @return Point2d[]
    */
   public ArrayList<FramePoint2d> getAllVisibleVerticesFromOutsideLeftToRight(FramePoint2d observerFramePoint)
   {
      this.checkReferenceFrameMatch(observerFramePoint);

      ArrayList<FramePoint2d> ret = new ArrayList<FramePoint2d>();

      ArrayList<Point2d> vertices = this.convexPolygon.getAllVisibleVerticesFromOutsideLeftToRightCopy(observerFramePoint.getPointCopy());
      if (vertices == null)
         return null;

      for (Point2d vertex : vertices)
      {
         ret.add(new FramePoint2d(referenceFrame, vertex));
      }

      return ret;
   }

   public FramePoint2d[] getLineOfSightVertices(FramePoint2d observerFramePoint)
   {
     this.checkReferenceFrameMatch(observerFramePoint);  

      Point2d[] lineOfSightVertices2d = this.convexPolygon.getLineOfSightVerticesCopy(observerFramePoint.getPointCopy());
      if (lineOfSightVertices2d == null)
         return null; // Point must be inside!

      FramePoint2d[] ret = new FramePoint2d[] { new FramePoint2d(observerFramePoint.getReferenceFrame(), lineOfSightVertices2d[0]),
            new FramePoint2d(observerFramePoint.getReferenceFrame(), lineOfSightVertices2d[1]) };

      
      return ret;
   }

   /**
    * Returns the two FrameLineSegment2ds that are the first segments around the corner that cannot be seen from the observer FramePoint2d.
    * If the observer FramePoint2d is null returns null. The line segments are returned in order of left, then right.
    * The line segments go from the line of sight points to the points that are not in view, but are around the corner.
    *
    * @param observerFramePoint FramePoint2d marking the point of observation of this ConvexPolygon2d.
    * @return FrameLineSegment2d[] Two line segments going from the line of sight points to the first points around the corners that are out of sight.
    * null if the observer FramePoint2d is inside this FrameConvexPolygon2d.
    */
   public FrameLineSegment2d[] getAroundTheCornerEdges(FramePoint2d observerFramePoint)
   {
      this.checkReferenceFrameMatch(observerFramePoint);

      LineSegment2d[] aroundTheCornerLineSegments2d = this.convexPolygon.getAroundTheCornerEdgesCopy(observerFramePoint.getPointCopy());

      if (aroundTheCornerLineSegments2d == null)
         return null;

      FrameLineSegment2d[] ret = new FrameLineSegment2d[] { new FrameLineSegment2d(observerFramePoint.getReferenceFrame(), aroundTheCornerLineSegments2d[0]),
            new FrameLineSegment2d(observerFramePoint.getReferenceFrame(), aroundTheCornerLineSegments2d[1]) };

      return ret;
   }

   public FrameLineSegment2d[] getLineSegmentsOfSight(FramePoint2d observerFramePoint)
   {
      FramePoint2d[] lineOfSightVertices = getLineOfSightVertices(observerFramePoint);

      return new FrameLineSegment2d[] { new FrameLineSegment2d(observerFramePoint, lineOfSightVertices[0]),
            new FrameLineSegment2d(observerFramePoint, lineOfSightVertices[1]) };
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

      LineSegment2d[] lineSegments = convexPolygon.getIntersectingEdges(frameLine2d.getLine2dCopy());
      if (lineSegments == null)
         return null;

      FrameLineSegment2d[] ret = new FrameLineSegment2d[lineSegments.length];

      for (int i = 0; i < lineSegments.length; i++)
      {
         ret[i] = new FrameLineSegment2d(referenceFrame, lineSegments[i]);
      }

      return ret;
   }

   public double[] distanceToEachVertex(FramePoint2d point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      return convexPolygon.distanceToEachVertex(point.getPoint());
   }

   public FramePoint2d getClosestVertexCopy(FramePoint2d point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      return new FramePoint2d(referenceFrame, convexPolygon.getClosestVertexCopy(point.getPoint()));
   }

   public FramePoint2d getClosestVertexWithRayCopy(FrameLine2d ray, boolean throwAwayVerticesOutsideRay)
   {
      ray.checkReferenceFrameMatch(referenceFrame);

      return new FramePoint2d(referenceFrame, convexPolygon.getClosestVertexWithRayCopy(ray.getLine2d(), throwAwayVerticesOutsideRay));
   }

   public boolean getClosestVertexWithRay(FramePoint2d closestVertexToPack, FrameLine2d ray, boolean throwAwayVerticesOutsideRay)
   {
      ray.checkReferenceFrameMatch(referenceFrame);
      closestVertexToPack.setToZero(referenceFrame);
      boolean success = convexPolygon.getClosestVertexWithRay(closestVertexToPack.getPoint(), ray.getLine2d(), throwAwayVerticesOutsideRay);

      return success;
   }

   public void getClosestVertex(FramePoint2d closestVertexToPack, FramePoint2d point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      closestVertexToPack.setIncludingFrame(referenceFrame, convexPolygon.getClosestVertexCopy(point.getPoint()));
   }

   public FramePoint2d getClosestVertexCopy(FrameLine2d line)
   {
      line.checkReferenceFrameMatch(referenceFrame);

      Point2d closestVertexCopy = convexPolygon.getClosestVertexCopy(line.line);
      
      if (closestVertexCopy == null)
         throw new RuntimeException("Closest vertex could not be found! Has at least one vertex: " + convexPolygon.hasAtLeastOneVertex());
      
      return new FramePoint2d(referenceFrame, closestVertexCopy);
   }

   public double distanceToClosestVertex(FramePoint2d point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      return convexPolygon.distanceToClosestVertex(point.getPoint());
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      convexPolygon.applyTransform(transform);
      updateFramePoints();
   }

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      convexPolygon.applyTransformAndProjectToXYPlane(transform);
      updateFramePoints();
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
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      referenceFrame = desiredFrame;
      applyTransform(temporaryTransformToDesiredFrame);
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
      String ret = "";
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         FramePoint2d vertex = frameVertices.get(i);
         ret = ret + vertex.toString() + "\n";
      }

      return ret;
   }

   public FrameLineSegment2d[] getNearestEdges(FramePoint2d testPoint)
   {
      checkReferenceFrameMatch(testPoint);

      LineSegment2d[] edges = convexPolygon.getNearestEdges(testPoint.getPoint());

      FrameLineSegment2d[] ret = new FrameLineSegment2d[edges.length];

      for (int i = 0; i < edges.length; i++)
      {
         ret[i] = new FrameLineSegment2d(referenceFrame, edges[i]);
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
 
   public void getOutwardEdgeNormals(FrameVector2d[] normalsToPack)
   {
      renewTemporaryVectorArray();
      
      convexPolygon.getOutSideFacingOrthoNormalVectors(temporaryVectorArray);
      
      for (int i = 0; i < normalsToPack.length; i++)
      {
         checkReferenceFrameMatch(normalsToPack[i]);
         normalsToPack[i].set(temporaryVectorArray[i]);
      }
   }
   
   private void renewTemporaryVectorArray()
   {
      if (temporaryVectorArray == null || temporaryVectorArray.length != getNumberOfVertices())
      {
         temporaryVectorArray = new Vector2d[getNumberOfVertices()];
         for (int i = 0; i < temporaryVectorArray.length; i++)
         {
            temporaryVectorArray[i] = new Vector2d();
         }
      }
   }

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

   public FramePoint2d[] intersectionWithRay(FrameLine2d ray)
   {
      checkReferenceFrameMatch(ray);
      Point2d[] intersections = convexPolygon.intersectionWithRay(ray.getLine2dCopy());
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
      intersectionToPack.updateFramePoints();

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

   public double perimeter()
   {
      return convexPolygon.perimeter();
   }

   public FramePoint2d pointOnPerimeterGivenParameter(double parameter)
   {
      return new FramePoint2d(referenceFrame, convexPolygon.pointOnPerimeterGivenParameter(parameter));
   }

   public void pullPointTowardsCentroid(FramePoint2d point, double percent)
   {
      checkReferenceFrameMatch(point);
      Point2d point2d = point.getPoint();
      convexPolygon.pullPointTowardsCentroid(point2d, percent);
   }

   public boolean isUpToDate()
   {
      return convexPolygon.isUpToDate();
   }

   public boolean isEmpty()
   {
      return convexPolygon.isEmpty();
   }
}
