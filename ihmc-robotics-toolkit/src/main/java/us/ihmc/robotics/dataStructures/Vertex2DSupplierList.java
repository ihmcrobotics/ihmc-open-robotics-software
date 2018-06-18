package us.ihmc.robotics.dataStructures;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class Vertex2DSupplierList<T extends Point2DReadOnly> implements Vertex2DSupplier, List<T>
{
   private int firstVertexIndex;
   private int lastVertexIndex;
   private final boolean useVertexIndexingAsVertexSupplier;
   private final List<T> vertexList;

   public Vertex2DSupplierList(boolean useVertexIndexing)
   {
      vertexList = new ArrayList<>();
      useVertexIndexingAsVertexSupplier = useVertexIndexing;
   }
   
   public Vertex2DSupplierList()
   {
      this(false);
   }

   public Vertex2DSupplierList(List<T> backingList, boolean useVertexIndexing)
   {
      vertexList = backingList;
      useVertexIndexingAsVertexSupplier = useVertexIndexing;
   }
   
   public Vertex2DSupplierList(List<T> backingList)
   {
      this(backingList, false);
   }

   public List<T> getVertexList()
   {
      return vertexList;
   }
   
   public void setIndicesForVerticesToSupply(int firstIndex, int lastIndex)
   {
      checkIfVertexIndexingIsEnabled();
      if (firstIndex > lastIndex)
         throw new RuntimeException("The first index (" + firstIndex +") cannot be less than last index (" + lastIndex + ")");
      firstVertexIndex = firstIndex;
      lastVertexIndex = lastIndex;
   }

   public void setIndicesFromList()
   {
      checkIfVertexIndexingIsEnabled();
      firstVertexIndex = 0;
      lastVertexIndex = vertexList.size() - 1;
   }
   
   private void checkIfVertexIndexingIsEnabled()
   {
      if(!useVertexIndexingAsVertexSupplier)
         throw new RuntimeException("Cannot use vertex index. Option was disabled at instantiation");
   }
   
   private void checkIfVertexIndexIsValid(int indexToCheck)
   {
      checkIfVertexIndexingIsEnabled();
      if(indexToCheck < 0 && indexToCheck >= getNumberOfVertices())
         throw new IndexOutOfBoundsException("Index (" + indexToCheck + ") is not in bounds");
   }

   @Override
   public T getVertex(int index)
   {
      if(useVertexIndexingAsVertexSupplier)
      {
         checkIfVertexIndexIsValid(index);
         return vertexList.get(firstVertexIndex + index);
      }
      else
         return vertexList.get(index);
   }

   @Override
   public int getNumberOfVertices()
   {
      if (useVertexIndexingAsVertexSupplier)
         return lastVertexIndex - firstVertexIndex + 1;
      else
         return vertexList.size();
   }

   @Override
   public boolean add(T pointToAdd)
   {
      return vertexList.add(pointToAdd);
   }

   @Override
   public void add(int indexToInsertAt, T pointToInsert)
   {
      vertexList.add(indexToInsertAt, pointToInsert);
   }

   @Override
   public void clear()
   {
      vertexList.clear();
   }

   @Override
   public boolean contains(Object objectToCheck)
   {
      return vertexList.contains(objectToCheck);
   }

   @Override
   public boolean containsAll(Collection<?> collectionToCheck)
   {
      return vertexList.contains(collectionToCheck);
   }

   @Override
   public T get(int index)
   {
      return vertexList.get(index);
   }

   @Override
   public int indexOf(Object object)
   {
      return vertexList.indexOf(object);
   }

   @Override
   public boolean isEmpty()
   {
      return vertexList.isEmpty();
   }

   @Override
   public Iterator<T> iterator()
   {
      return vertexList.iterator();
   }

   @Override
   public int lastIndexOf(Object object)
   {
      return vertexList.lastIndexOf(object);
   }

   @Override
   public ListIterator<T> listIterator()
   {
      return vertexList.listIterator();
   }

   @Override
   public ListIterator<T> listIterator(int index)
   {
      return null;
   }

   @Override
   public boolean remove(Object object)
   {
      return vertexList.remove(object);
   }

   @Override
   public T remove(int index)
   {
      return vertexList.remove(index);
   }

   @Override
   public boolean removeAll(Collection<?> collection)
   {
      return vertexList.removeAll(collection);
   }

   @Override
   public boolean retainAll(Collection<?> collection)
   {
      return vertexList.retainAll(collection);
   }

   @Override
   public T set(int index, T element)
   {
      return vertexList.set(index, element);
   }

   @Override
   public int size()
   {
      return vertexList.size();
   }

   @Override
   public List<T> subList(int fromIndex, int toIndex)
   {
      return vertexList.subList(fromIndex, toIndex);
   }

   @Override
   public Object[] toArray()
   {
      return vertexList.toArray();
   }

   @Override
   public <T> T[] toArray(T[] arrayToConvert)
   {
      return vertexList.toArray(arrayToConvert);
   }

   @Override
   public boolean addAll(Collection<? extends T> c)
   {
      return vertexList.addAll(c);
   }

   @Override
   public boolean addAll(int index, Collection<? extends T> c)
   {
      return vertexList.addAll(index, c);
   }
}
