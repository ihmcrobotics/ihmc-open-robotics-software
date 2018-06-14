package us.ihmc.robotics.dataStructures;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Vertex3DSupplierList<T extends Point3DReadOnly> implements Vertex3DSupplier, List<T>
{
   protected int firstVertexIndex;
   protected int lastVertexIndex;
   protected final boolean useVertexIndexingAsVertexSupplier;
   protected final List<T> vertexList;

   public Vertex3DSupplierList(boolean useVertexIndexing)
   {
      vertexList = new ArrayList<>();
      useVertexIndexingAsVertexSupplier = useVertexIndexing;
   }
   
   public Vertex3DSupplierList()
   {
      this(false);
   }

   public Vertex3DSupplierList(List<T> backingList, boolean useVertexIndexing)
   {
      vertexList = backingList;
      useVertexIndexingAsVertexSupplier = useVertexIndexing;
   }
   
   public Vertex3DSupplierList(List<T> backingList)
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
   
   public void checkIfVertexIndexingIsEnabled()
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
   public Point3DReadOnly getVertex(int index)
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
   public boolean add(T arg0)
   {
      return vertexList.add(arg0);
   }

   @Override
   public void add(int arg0, T arg1)
   {
      vertexList.add(arg0, arg1);
   }

   @Override
   public void clear()
   {
      vertexList.clear();
   }

   @Override
   public boolean contains(Object arg0)
   {
      return vertexList.contains(arg0);
   }

   @Override
   public boolean containsAll(Collection<?> arg0)
   {
      return vertexList.contains(arg0);
   }

   @Override
   public T get(int arg0)
   {
      return vertexList.get(arg0);
   }

   @Override
   public int indexOf(Object arg0)
   {
      return vertexList.indexOf(arg0);
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
   public int lastIndexOf(Object o)
   {
      return vertexList.lastIndexOf(o);
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
   public boolean remove(Object o)
   {
      return vertexList.remove(o);
   }

   @Override
   public T remove(int index)
   {
      return vertexList.remove(index);
   }

   @Override
   public boolean removeAll(Collection<?> c)
   {
      return vertexList.removeAll(c);
   }

   @Override
   public boolean retainAll(Collection<?> c)
   {
      return vertexList.retainAll(c);
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
   public <T> T[] toArray(T[] a)
   {
      return vertexList.toArray(a);
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
