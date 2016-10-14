package us.ihmc.communication.controllerAPI.command;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Iterator;

import us.ihmc.robotics.lists.GenericTypeBuilder;

public class CommandArrayDeque<C extends Command<C, ?>> extends ArrayDeque<C>
{
   private static final long serialVersionUID = 8118722036566615731L;

   private final GenericTypeBuilder<C> commandBuilder;
   private final ArrayDeque<C> unusedCommands;
   
   public CommandArrayDeque(Class<C> commandClass)
   {
      this(16, commandClass);
   }

   public CommandArrayDeque(int numElements, Class<C> commandClass)
   {
      super(numElements);
      commandBuilder = GenericTypeBuilder.createBuilderWithEmptyConstructor(commandClass);
      unusedCommands = new ArrayDeque<>(numElements);
      for (int i = 0; i < numElements; i++)
         unusedCommands.add(commandBuilder.newInstance());
   }

   /** {@inheritDoc} */
   @Override
   public int size()
   {
      return super.size();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isEmpty()
   {
      return super.isEmpty();
   }

   /**
    * Add an empty command at the front of this deque and return it.
    * @return the new empty command.
    */
   public C addFirst()
   {
      C newCommand = getOrCreateUnusedCommand();
      newCommand.clear();
      super.addFirst(newCommand);
      return newCommand;
   }

   /**
    * Add an empty command at the end of this deque and return it.
    * @return the new empty command.
    */
   public C addLast()
   {
      C newCommand = getOrCreateUnusedCommand();
      newCommand.clear();
      super.addLast(newCommand);
      return newCommand;
   }

   /** {@inheritDoc} */
   @Override
   public boolean add(C newCommand)
   {
      return super.add(copyAndReturnLocalCommand(newCommand));
   }

   /**
    * The deque will be empty after this call returns.
    * The removed elements are saved in a local buffer for recycling purpose to prevent garbage generation.
    */
   @Override
   public void clear()
   {
      while (!super.isEmpty())
         unusedCommands.add(super.poll());
   }

   /** {@inheritDoc} */
   @Override
   public void addFirst(C newCommand)
   {
      super.addFirst(copyAndReturnLocalCommand(newCommand));
   }

   /** {@inheritDoc} */
   @Override
   public void addLast(C newCommand)
   {
      super.addLast(copyAndReturnLocalCommand(newCommand));
   }

   /** {@inheritDoc} */
   @Override
   public void push(C newCommand)
   {
      super.push(copyAndReturnLocalCommand(newCommand));
   }

   /** {@inheritDoc} */
   @Override
   public boolean offerFirst(C newCommand)
   {
      return super.offerFirst(copyAndReturnLocalCommand(newCommand));
   }

   /** {@inheritDoc} */
   @Override
   public boolean offerLast(C newCommand)
   {
      return super.offerLast(copyAndReturnLocalCommand(newCommand));
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C removeFirst()
   {
      C commandToReturn = super.removeFirst();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C removeLast()
   {
      C commandToReturn = super.removeLast();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C pollFirst()
   {
      C commandToReturn = super.pollFirst();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C pollLast()
   {
      C commandToReturn = super.pollLast();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C getFirst()
   {
      C commandToReturn = super.getFirst();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C getLast()
   {
      C commandToReturn = super.getLast();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C peekFirst()
   {
      C commandToReturn = super.peekFirst();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C peekLast()
   {
      C commandToReturn = super.peekLast();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C remove()
   {
      C commandToReturn = super.remove();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C poll()
   {
      C commandToReturn = super.poll();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C element()
   {
      C commandToReturn = super.element();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C peek()
   {
      C commandToReturn = super.peek();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public C pop()
   {
      C commandToReturn = super.pop();
      unusedCommands.add(commandToReturn);
      return commandToReturn;
   }

   private C copyAndReturnLocalCommand(C commandToCopy)
   {
      C localCommand = getOrCreateUnusedCommand();
      localCommand.set(commandToCopy);
      return localCommand;
   }

   private C getOrCreateUnusedCommand()
   {
      if (unusedCommands.isEmpty())
         return commandBuilder.newInstance();
      else
         return unusedCommands.poll();
   }

   @Override
   public String toString()
   {
      Iterator<C> iterator = super.iterator();
      if (!iterator.hasNext())
         return "[]";

      StringBuilder sb = new StringBuilder();
      sb.append('[');
      for (;;)
      {
         C nextCommand = iterator.next();
         sb.append(nextCommand);
         if (!iterator.hasNext())
            return sb.append(']').toString();
         sb.append(',').append(' ');
      }
   }

   /** Unsupported operation. */
   @Override
   public CommandArrayDeque<C> clone()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean remove(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean contains(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Iterator<C> iterator()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Object[] toArray()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public <T> T[] toArray(T[] a)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean containsAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean addAll(Collection<? extends C> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean retainAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeFirstOccurrence(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeLastOccurrence(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean offer(C e)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Iterator<C> descendingIterator()
   {
      throw new UnsupportedOperationException();
   }
}
