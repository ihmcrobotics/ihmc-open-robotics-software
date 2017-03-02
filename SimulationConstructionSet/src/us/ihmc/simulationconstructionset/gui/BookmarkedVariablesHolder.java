package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class BookmarkedVariablesHolder
{
   private ArrayList<YoVariable<?>> bookmarkedVariables = new ArrayList<>(0);
   private ArrayList<BookmarkedVariableAddedListener> listeners = new ArrayList<BookmarkedVariableAddedListener>(0);
   private ArrayList<BookmarkedVariableRemovedListener> removedListeners = new ArrayList<BookmarkedVariableRemovedListener>(0);

   public ArrayList<YoVariable<?>> getBookMarkedVariables()
   {
      return new ArrayList<YoVariable<?>>(bookmarkedVariables);
   }

   public void addBookmark(YoVariable<?> variable)
   {
      if (!bookmarkedVariables.contains(variable))
      {
         bookmarkedVariables.add(variable);
         notifyBookmarkedVariableAddedListeners();
      }
   }

   public void removeBookmark(YoVariable<?> variable)
   {
      if (bookmarkedVariables.remove(variable))
      {
         notifyBookmarkedVariableRemovedListeners(variable);
      }
   }

   public void addBookmarkedVariableAddedListener(BookmarkedVariableAddedListener listener)
   {
      listeners.add(listener);
   }

   public void addBookmarkedVariableRemovedListener(BookmarkedVariableRemovedListener listener)
   {
      removedListeners.add(listener);
   }

   private void notifyBookmarkedVariableRemovedListeners(YoVariable<?> variable)
   {
      for (BookmarkedVariableRemovedListener removedListener : removedListeners)
      {
         removedListener.bookmarkRemoved(variable);
      }
   }

   private void notifyBookmarkedVariableAddedListeners()
   {
      for (BookmarkedVariableAddedListener listener : listeners)
      {
         listener.bookmarkAdded(bookmarkedVariables.get(bookmarkedVariables.size() - 1));
      }
   }
}
