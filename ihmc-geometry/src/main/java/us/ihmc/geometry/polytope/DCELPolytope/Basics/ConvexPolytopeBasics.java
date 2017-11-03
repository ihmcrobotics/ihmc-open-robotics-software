package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexProvider;

public abstract class ConvexPolytopeBasics<A extends PolytopeVertexBasics<A, B, C>, B extends PolytopeHalfEdgeBasics<A, B, C>, C extends ConvexPolytopeFaceBasics<A, B, C>>  implements ConvexPolytopeReadOnly, SimplexBasics, Clearable, Transformable, Settable<ConvexPolytopeReadOnly>
{
   private final ArrayList<A> vertices = new ArrayList<>();
   private final ArrayList<B> edges = new ArrayList<>();
   private final ArrayList<C> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final ArrayList<C> visibleFaces = new ArrayList<>();
   private final ArrayList<C> silhouetteFaces = new ArrayList<>();
   private final ArrayList<C> nonSilhouetteFaces = new ArrayList<>();
   private final ArrayList<C> onFaceList = new ArrayList<>();
   private final ArrayList<B> visibleSilhouetteList = new ArrayList<>();

   // Temporary variables for intermediate results
   private Vector3D tempVector = new Vector3D();
   private Point3D centroid = new Point3D();

   public ConvexPolytopeBasics()
   {
      
   }
   
   public ConvexPolytopeBasics(ConvexPolytopeReadOnly polytope)
   {
      set(polytope);
      boundingBoxNeedsUpdating = true;
   }

   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }
      boundingBoxToPack.set(boundingBox);
   }

   private void updateBoundingBox()
   {
      double xMin = Double.POSITIVE_INFINITY;
      double yMin = Double.POSITIVE_INFINITY;
      double zMin = Double.POSITIVE_INFINITY;

      double xMax = Double.NEGATIVE_INFINITY;
      double yMax = Double.NEGATIVE_INFINITY;
      double zMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < faces.size(); i++)
      {
         double x = faces.get(i).getMinX();
         double y = faces.get(i).getMinY();
         double z = faces.get(i).getMinZ();

         if (x < xMin)
            xMin = x;
         if (y < yMin)
            yMin = y;
         if (z < zMin)
            zMin = z;

         x = faces.get(i).getMaxX();
         y = faces.get(i).getMaxY();
         z = faces.get(i).getMaxZ();
         if (x > xMax)
            xMax = x;
         if (y > yMax)
            yMax = y;
         if (z > zMax)
            zMax = z;
      }
      boundingBox.set(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   public int getNumberOfVertices()
   {
      // Polyhedron formula for quick calc
      return getNumberOfEdges() - getNumberOfFaces() + 2;
   }

   public List<A> getVertices()
   {
      updateVertices();
      return vertices;
   }
   
   public void getVertices(ArrayList<Point3D> verticesToPack)
   {
      updateVertices();
      for(int i = 0; i < vertices.size(); i++)
         verticesToPack.get(i).set(vertices.get(i));
   }

   private void updateVertices()
   {
      unmarkAllFaces();
      vertices.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < faces.get(i).getNumberOfEdges(); j++)
         {
            if (!faces.get(i).getEdge(j).getOriginVertex().isAnyFaceMarked())
            {
               vertices.add(faces.get(i).getEdge(j).getOriginVertex());
            }
         }
         faces.get(i).mark();
      }
   }

   public A getVertex(int index)
   {
      updateVertices();
      return vertices.get(index);
   }

   public int getNumberOfEdges()
   {
      updateEdges();
      return edges.size() / 2;
   }

   public List<B> getEdges()
   {
      updateEdges();
      return edges;
   }

   private void updateEdges()
   {
      edges.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         List<B> faceEdgeList = faces.get(i).getEdgeList();
         for (int j = 0; j < faceEdgeList.size(); j++)
         {
            edges.add(faceEdgeList.get(j));
         }
      }
   }

   public int getNumberOfFaces()
   {
      return faces.size();
   }

   public List<C> getFaces()
   {
      return faces;
   }

   public C getFace(int index)
   {
      return faces.get(index);
   }
   
   public Vector3DReadOnly getFaceNormalAt(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getFaceNormal();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyTransform(transform);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyInverseTransform(transform);
      boundingBoxNeedsUpdating = true;
   }

   private void unmarkAllFaces()
   {
      for (int i = 0; i < faces.size(); i++)
         faces.get(i).unmark();
   }

   public void addVertices(double epsilon, Point3D... vertices)
   {
      for (int i = 0; i < vertices.length; i++)
         addVertex(vertices[i], epsilon);
   }

   public void addVertices(List<A> vertices, double epsilon)
   {
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), epsilon);
   }

   public void addVertex(double epsilon, double... coordinates)
   {
      addVertex(getVertexProvider().getVertex(coordinates[0], coordinates[1], coordinates[2]), epsilon);
   }

   public void addVertex(double x, double y, double z, double epsilon)
   {
      addVertex(getVertexProvider().getVertex(x, y, z), epsilon);
   }

   public void addVertex(Point3D vertexToAdd, double epsilon)
   {
      addVertex(getVertexProvider().getVertex(vertexToAdd), epsilon);
   }

   /**
    * Adds a polytope vertex to the current polytope. 
    * In case needed faces are removed and recreated. This will result in garbage. Fix if possible
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public void addVertex(A vertexToAdd, double epsilon)
   {
      if (faces.size() == 0)
      {
         // Polytope is empty. Creating face and adding the vertex
         C newFace = getConvexFaceProvider().getFace();
         newFace.addVertex(vertexToAdd);
         faces.add(newFace);
         boundingBoxNeedsUpdating = true;
         return;
      }
      else if (faces.size() == 1)
      {
         if (faces.get(0).isPointInFacePlane(vertexToAdd, Epsilons.ONE_MILLIONTH))
         {
            if (!faces.get(0).isInteriorPoint(vertexToAdd))
               faces.get(0).addVertex(vertexToAdd);
            return;
         }
         else
         {
            if (faces.get(0).isFaceVisible(vertexToAdd, epsilon))
               faces.get(0).reverseFaceNormal();

            visibleSilhouetteList.clear();
            B halfEdge = faces.get(0).getEdge(0);
            for (int i = 0; i < faces.get(0).getNumberOfEdges(); i++)
            {
               visibleSilhouetteList.add(halfEdge);
               halfEdge = halfEdge.getPreviousHalfEdge();
            }
            onFaceList.clear();
            createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd);
         }
         boundingBoxNeedsUpdating = true;
         return;
      }
      C visibleFaceSeed = getVisibleFaces(visibleFaces, vertexToAdd, epsilon);
      if (visibleFaces.isEmpty())
         return;
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, Epsilons.ONE_BILLIONTH);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      B firstHalfEdgeForSilhouette = onFaceList.size() > 0 ? onFaceList.get(0).getFirstVisibleEdge(vertexToAdd).getTwinHalfEdge()
            : getSeedEdgeForSilhouetteCalculation(visibleFaces, visibleFaceSeed);
      // remove this hack
      if(firstHalfEdgeForSilhouette == null)
      {
         PrintTools.warn("Got null seed for face construction. Skipping this point.");
         return;
      }
      getVisibleSilhouetteUsingSeed(visibleSilhouetteList, firstHalfEdgeForSilhouette, visibleFaces);
      removeFaces(nonSilhouetteFaces);
      removeFaces(silhouetteFaces);
      createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd);
      boundingBoxNeedsUpdating = true;
   }

   private double getTripleProduct(Vector3DReadOnly v1, Vector3DReadOnly v2, Vector3DReadOnly v3)
   {
      tempVector.cross(v1, v2);
      return tempVector.dot(v3);
   }

   private B getCommonEdge(C face1, C face2)
   {
      B edgeCandidate = null;
      tempVector.cross(face1.getFaceNormal(), face2.getFaceNormal());
      tempVector.normalize();
      for (int i = 0; i < face1.getNumberOfEdges(); i++)
      {
         if (tempVector.dot(face1.getEdge(i).getNormalizedEdgeVector()) >= 1.0)
         {
            edgeCandidate = face1.getEdge(i);
            break;
         }
      }
      return edgeCandidate;
   }

   public void getSilhouetteFaces(List<C> silhouetteFacesToPack, List<C> nonSilhouetteFacesToPack,
                                  List<C> visibleFaceList)
   {
      if (silhouetteFacesToPack != null)
         silhouetteFacesToPack.clear();
      if (nonSilhouetteFacesToPack != null)
         nonSilhouetteFacesToPack.clear();
      for (int i = 0; i < visibleFaceList.size(); i++)
      {
         C candidateFace = visibleFaceList.get(i);

         boolean allNeighbouringFacesVisible = true;
         for (int j = 0; j < candidateFace.getNumberOfEdges(); j++)
            allNeighbouringFacesVisible &= visibleFaceList.contains(candidateFace.getNeighbouringFace(j));

         if (allNeighbouringFacesVisible && nonSilhouetteFacesToPack != null)
            nonSilhouetteFacesToPack.add(candidateFace);
         else if (silhouetteFacesToPack != null)
            silhouetteFacesToPack.add(candidateFace);
      }
   }

   public void getVisibleSilhouette(Point3DReadOnly vertex, List<B> visibleSilhouetteToPack, double epsilon)
   {
      C leastVisibleFace = getVisibleFaces(visibleFaces, vertex, epsilon);
      if (visibleFaces.isEmpty())
         return;
      getFacesWhichPointIsOn(vertex, onFaceList, Epsilons.ONE_BILLIONTH);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      B firstHalfEdgeForSilhouette = onFaceList.size() > 0 ? onFaceList.get(0).getFirstVisibleEdge(vertex).getTwinHalfEdge()
            : getSeedEdgeForSilhouetteCalculation(visibleFaces, leastVisibleFace);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteToPack, firstHalfEdgeForSilhouette, visibleFaces);
   }

   public void getVisibleSilhouetteUsingSeed(List<B> visibleSilhouetteToPack, B seedHalfEdge,
                                             List<C> silhouetteFaceList)
   {
      B halfEdgeUnderConsideration = seedHalfEdge;
      visibleSilhouetteToPack.clear();
      int numberOfEdges = getNumberOfEdges();
      int count;
      for (count = 0; count < numberOfEdges; count++)
      {
         if(halfEdgeUnderConsideration == null)
            PrintTools.debug("Half edge null " + faces.size());
         if(visibleSilhouetteToPack == null)
            PrintTools.debug("visible list null");
         if(halfEdgeUnderConsideration.getTwinHalfEdge() == null)
            PrintTools.debug("Twing half edge null");

         visibleSilhouetteToPack.add(halfEdgeUnderConsideration.getTwinHalfEdge());
         A destinationVertex = halfEdgeUnderConsideration.getDestinationVertex();
         for (int i = 0; i < destinationVertex.getNumberOfAssociatedEdges(); i++)
         {
            if (silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getFace())
                  && !silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getTwinHalfEdge().getFace()))
            {
               halfEdgeUnderConsideration = destinationVertex.getAssociatedEdge(i);
               break;
            }
         }
         if (halfEdgeUnderConsideration == seedHalfEdge)
            break;
      }
      if (count == numberOfEdges)
      {
         PrintTools.error("Could not determine visible silhouette \n" + toString());
//         PrintTools.warn("On face size: " + onFaceList.size());
//         for(int i = 0; i < onFaceList.size(); i++)
//            PrintTools.debug(onFaceList.get(i).toString());
//         PrintTools.warn("Visible face size: " + visibleFaces.size());
//         for(int i = 0; i < visibleFaces.size(); i++)
//            PrintTools.debug(visibleFaces.get(i).toString());
         faces.clear();
      }
   }

   public B getSeedEdgeForSilhouetteCalculation(List<C> visibleFaceList, C leastVisibleFace)
   {
      if (faces.size() == 1)
         return faces.get(0).getEdge(0);
      B seedEdge = null;
      B seedEdgeCandidate = leastVisibleFace.getEdge(0);
      for (int i = 0; seedEdge == null && i < leastVisibleFace.getNumberOfEdges(); i++)
      {
         if (!visibleFaceList.contains(seedEdgeCandidate.getTwinHalfEdge().getFace()))
            seedEdge = seedEdgeCandidate;
         seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
      }
      return seedEdge;
   }

   //   public S getSeedEdgeForSilhouetteCalculation(List<U> silhouetteFaceList)
   //   {
   //      U seedFaceCandidate = silhouetteFaceList.get(0);
   //      S seedEdgeCandidate = seedFaceCandidate.getEdge(0);
   //      for (int i = 0; i < seedFaceCandidate.getNumberOfEdges(); i++)
   //      {
   //         if (silhouetteFaceList.contains(seedEdgeCandidate.getTwinHalfEdge().getFace())
   //               && !silhouetteFaceList.contains(seedEdgeCandidate.getNextHalfEdge().getTwinHalfEdge().getFace()))
   //            break;
   //         seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
   //      }
   //      seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
   //      return seedEdgeCandidate;
   //   }

   //   private void createFacesFromVisibleSilhouette(PolytopeVertex vertexToAdd)
   //   {
   //      U firstNewFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(0));
   //      twinEdges(visibleSilhouetteList.get(0), firstNewFace.getEdge(0));
   //      for (int i = 1; i < visibleSilhouetteList.size(); i++)
   //      {
   //         U newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(i));
   //         twinEdges(visibleSilhouetteList.get(i - 1).getTwinHalfEdge().getNextHalfEdge(), newFace.getEdge(0).getPreviousHalfEdge());
   //      }
   //      twinEdges(visibleSilhouetteList.get(visibleSilhouetteList.size() - 1).getTwinHalfEdge().getNextHalfEdge(), firstNewFace.getEdge(0).getPreviousHalfEdge());
   //   }

   private void createFacesFromVisibleSilhouetteAndOnFaceList(List<B> silhouetteEdges, List<C> onFaceList, A vertexToAdd)
   {
      //for(int i = 0; i < silhouetteEdges.size(); i++)
      //PrintTools.debug("Sil: " + silhouetteEdges.get(i));
      //PrintTools.debug(silhouetteEdges.get(0).getFace().toString());
      B previousLeadingEdge = null, trailingEdge = null;
      if (onFaceList.contains(silhouetteEdges.get(0).getFace()))
      {
         previousLeadingEdge = silhouetteEdges.get(0).getFace().getFirstVisibleEdge(vertexToAdd);
         silhouetteEdges.get(0).getFace().addVertex(vertexToAdd);
         trailingEdge = previousLeadingEdge.getNextHalfEdge();
      }
      else
      {
         C firstFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(0));
         previousLeadingEdge = firstFace.getEdge(0).getNextHalfEdge();
         trailingEdge = firstFace.getEdge(0).getPreviousHalfEdge();
         //PrintTools.debug("This happened");
      }
      //PrintTools.debug("PrevLeadEdge: "  + ((previousLeadingEdge == null )? "null" : previousLeadingEdge.toString()));
      //PrintTools.debug("TrailEdge: "  + ((trailingEdge == null) ? "null" : trailingEdge.toString() ));
      for (int i = 1; i < silhouetteEdges.size(); i++)
      {
         //PrintTools.debug("Previous leading: " + previousLeadingEdge.toString() + " Visible : " + visibleSilhouetteList.get(i).toString());
         if (onFaceList.contains(silhouetteEdges.get(i).getFace()))
         {
            B tempEdge = silhouetteEdges.get(i).getFace().getFirstVisibleEdge(vertexToAdd);
            silhouetteEdges.get(i).getFace().addVertex(vertexToAdd);
            twinEdges(previousLeadingEdge, tempEdge.getNextHalfEdge());
            previousLeadingEdge = tempEdge;
         }
         else
         {
            C newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(i));
            twinEdges(previousLeadingEdge, newFace.getEdge(0).getPreviousHalfEdge());
            previousLeadingEdge = newFace.getEdge(0).getNextHalfEdge();
         }
      }
      twinEdges(previousLeadingEdge, trailingEdge);
   }

   private void twinEdges(B halfEdge1, B halfEdge2)
   {
      halfEdge1.setTwinHalfEdge(halfEdge2);
      halfEdge2.setTwinHalfEdge(halfEdge1);
   }

   private C createFaceFromTwinEdgeAndVertex(A vertex, B twinEdge)
   {
      C newFace = getConvexFaceProvider().getFace();
      faces.add(newFace);
      newFace.addVertex(twinEdge.getDestinationVertex());
      newFace.addVertex(twinEdge.getOriginVertex());
      newFace.addVertex(vertex);
      twinEdges(newFace.getEdge(0), twinEdge);
      return newFace;
   }

   private void removeFaces(List<C> facesToRemove)
   {
      for (int i = 0; i < facesToRemove.size(); i++)
      {
         removeFace(facesToRemove.get(i));
      }
   }

   public C getVisibleFaces(List<C> faceReferencesToPack, Point3DReadOnly vertexUnderConsideration, double epsilon)
   {
      C leastVisibleFace = null;
      double minimumVisibilityProduct = Double.POSITIVE_INFINITY;
      faceReferencesToPack.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         double visibilityProduct = faces.get(i).getFaceVisibilityProduct(vertexUnderConsideration);
         if (visibilityProduct > epsilon)
         {
            faceReferencesToPack.add(faces.get(i));
            if (visibilityProduct < minimumVisibilityProduct)
            {
               leastVisibleFace = faces.get(i);
               minimumVisibilityProduct = visibilityProduct;
            }
         }
      }
      return leastVisibleFace;
   }

   public void getFacesWhichPointIsOn(Point3DReadOnly vertexUnderConsideration, List<C> faceReferenceToPack, double epsilon)
   {
      faceReferenceToPack.clear();

      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).isPointInFacePlane(vertexUnderConsideration, epsilon))
         {
            faceReferenceToPack.add(faces.get(i));
         }
      }
   }

   public void removeFace(C faceToRemove)
   {
      for (int i = 0; i < faceToRemove.getNumberOfEdges(); i++)
      {
         B twinHalfEdge = faceToRemove.getEdge(i).getTwinHalfEdge();
         if (twinHalfEdge != null)
            twinHalfEdge.setTwinHalfEdge(null);
         faceToRemove.getEdge(i).clear();
      }
      faceToRemove.clearEdgeList();
      faces.remove(faceToRemove);
   }

   private C isInteriorPointInternal(Point3DReadOnly pointToCheck, double epsilon)
   {
      if (faces.size() == 0)
         return null;
      else if (faces.size() == 1)
         return faces.get(0).isInteriorPoint(pointToCheck) ? null : faces.get(0);

      for (int i = 0; i < faces.size(); i++)
      {
         tempVector.sub(pointToCheck, faces.get(i).getEdge(0).getOriginVertex().getPosition());
         double dotProduct = tempVector.dot(faces.get(i).getFaceNormal());
         if (dotProduct >= epsilon || faces.get(i).getNumberOfEdges() < 3)
         {
            return faces.get(i);
         }
      }
      return null;
   }

   public boolean isInteriorPoint(Point3DReadOnly pointToCheck, double epsilon)
   {
      return isInteriorPointInternal(pointToCheck, epsilon) == null;
   }

   @Override
   public Point3D getSupportingVertex(Vector3D supportDirection)
   {
      A bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      A vertexCandidate = bestVertex;

      while (true)
      {
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            tempVector.set(bestVertex.getAssociatedEdge(i).getDestinationVertex());
            double dotProduct = supportDirection.dot(tempVector);
            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
               maxDotProduct = dotProduct;
            }
         }
         if (bestVertex == vertexCandidate)
            return (Point3D) bestVertex.getPosition();
         else
            bestVertex = vertexCandidate;
      }
   }

   // TODO Hacking this for the new collision detector. #FIXME fix this and the related interfaces and all that depends on those interfaces so that there dont need to be two versions of the getSupportinVertex function
   @Override
   public A getSupportingVertexHack(Vector3DReadOnly supportDirection)
   {
      A bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      A vertexCandidate = bestVertex;
      while (true)
      {
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            tempVector.set(bestVertex.getAssociatedEdge(i).getDestinationVertex());
            double dotProduct = supportDirection.dot(tempVector);
            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
               maxDotProduct = dotProduct;
            }
         }
         if (bestVertex == vertexCandidate)
            return bestVertex;
         else
            bestVertex = vertexCandidate;
      }
   }

   public String toString()
   {
      String string = "\n\nNumber of faces: " + faces.size();
      for (int i = 0; i < faces.size(); i++)
      {
         string = string + "\n" + faces.get(i).toString();
      }
      return string;
   }
   

   @Override
   public boolean epsilonEquals(ConvexPolytopeReadOnly other, double epsilon)
   {
      // TODO imlepment this
      return false;
   }

   @Override
   public boolean containsNaN()
   {
      boolean result = false;
      for (int i = 0; i < faces.size(); i++)
      {
         result |= faces.get(i).containsNaN();
      }
      return result;
   }

   @Override
   public void setToNaN()
   {
      // This should also set all the edges and vertices to NaN assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToNaN();
      }
   }

   @Override
   public void setToZero()
   {
      // This should also set all the edges and vertices to zero assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToZero();
      }
   }

   @Override
   public void set(ConvexPolytopeReadOnly other)
   {
      copyFaces(other.getFaces());
   }

   private void copyFaces(List<? extends ConvexPolytopeFaceReadOnly> faces)
   {
      //TODO implement this 
      throw new RuntimeException("Unimplemented feature");
   }

   public void clear()
   {
      vertices.clear();
      edges.clear();
      faces.clear();
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      if (isInteriorPoint(point, Epsilons.ONE_TRILLIONTH))
      {
         return getFaceContainingPointClosestTo(point).getShortestDistanceTo(point);
      }
      else
      {
         return getFaceContainingPointClosestTo(point).getShortestDistanceTo(point);
      }
   }

   public C getFaceContainingPointClosestTo(Point3DReadOnly point)
   {
      if (faces.size() == 0)
         throw new RuntimeException();
      else if (faces.size() == 1)
      {
         return faces.get(0);
      }

      unmarkAllFaces();
      C currentBestFace = faces.get(0);
      C faceUnderConsideration = currentBestFace;
      double minDistance = faceUnderConsideration.getShortestDistanceTo(point);
      faceUnderConsideration.mark();

      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < currentBestFace.getNumberOfEdges(); j++)
         {
            if (currentBestFace.getNeighbouringFace(j) != null && currentBestFace.getNeighbouringFace(j).isNotMarked())
            {
               double distance = currentBestFace.getNeighbouringFace(j).getShortestDistanceTo(point);
               if (distance < minDistance)
               {
                  minDistance = distance;
                  faceUnderConsideration = currentBestFace.getNeighbouringFace(j);
               }
               currentBestFace.getNeighbouringFace(j).mark();
            }
         }
         if (faceUnderConsideration == currentBestFace)
            break;
         else
            currentBestFace = faceUnderConsideration;
      }
      return currentBestFace;
   }

   private void updateCentroid()
   {
      updateVertices();
      centroid.setToZero();
      for (int i = 0; i < vertices.size(); i++)
         centroid.add(vertices.get(i));
      centroid.scale(1.0 / vertices.size());
   }

   public Point3DReadOnly getCentroid()
   {
      updateCentroid();
      return centroid;
   }

   protected abstract PolytopeVertexProvider<A,B,C> getVertexProvider();
   protected abstract ConvexPolytopeFaceProvider<A,B,C> getConvexFaceProvider();
   
   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      getFaceContainingPointClosestTo(point).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getSmallestSimplexMemberReference(point);
   }
}
