package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConcaveHullCollection implements Iterable<ConcaveHull>
{
   private final Set<ConcaveHull> concaveHulls = new HashSet<>();

   public ConcaveHullCollection()
   {
   }

   public ConcaveHullCollection(ConcaveHull concaveHull)
   {
      add(concaveHull);
   }

   public ConcaveHullCollection(ConcaveHull... concaveHulls)
   {
      addAll(concaveHulls);
   }

   public ConcaveHullCollection(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      add(concaveHullVertices);
   }

   public ConcaveHullCollection(ConcaveHullCollection other)
   {
      other.forEach(concaveHull -> add(new ConcaveHull(concaveHull)));
   }

   public boolean add(List<? extends Point2DReadOnly> newConcaveHullVertices)
   {
      if (newConcaveHullVertices == null)
         return false;
      else
         return add(new ConcaveHull(newConcaveHullVertices));
   }

   public boolean add(ConcaveHull newConcaveHull)
   {
      if (newConcaveHull == null)
         return false;
      else
         return concaveHulls.add(newConcaveHull);
   }

   public boolean addAll(ConcaveHull... newConcaveHulls)
   {
      if (newConcaveHulls == null)
         return false;
      
      boolean modified = false;
      
      for (ConcaveHull concaveHull : newConcaveHulls)
         modified |= add(concaveHull);
      
      return modified;
   }
   
   public boolean addAll(ConcaveHullCollection other)
   {
      if (other == null)
         return false;
      else
         return concaveHulls.addAll(other.concaveHulls);
   }

   public boolean remove(ConcaveHull newConcaveHull)
   {
      return concaveHulls.remove(newConcaveHull);
   }

   public int getNumberOfConcaveHulls()
   {
      return concaveHulls.size();
   }

   public boolean isEmpty()
   {
      return concaveHulls.isEmpty();
   }

   public Collection<ConcaveHull> getConcaveHulls()
   {
      return concaveHulls;
   }

   @Override
   public Iterator<ConcaveHull> iterator()
   {
      return concaveHulls.iterator();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConcaveHullCollection)
         return concaveHulls.equals(((ConcaveHullCollection) object).concaveHulls);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      int hashCode = 1;
      for (ConcaveHull hull : this)
         hashCode = 31 * hashCode + (hull == null ? 0 : hull.hashCode());
      return hashCode;
   }
}
