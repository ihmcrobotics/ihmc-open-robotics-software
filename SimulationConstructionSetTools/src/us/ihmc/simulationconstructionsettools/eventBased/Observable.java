package us.ihmc.simulationconstructionsettools.eventBased;


/**
 * Interface for all classes that are meant to be observed by instances of a class that implements Observer.
 * Observers should be notified by the Observable when deemed appropriate.
 *
 * @author Twan Koolen
 *
 */
public interface Observable
{
   /**
    * Adds a new observer
    * @param observer the new observer to be addedd
    */
   public void addObserver(Observer observer);

   /**
    * Removes an observer
    * @param observer the new observer to be removed
    * @return true iff the observer was removed from the list
    */
   public void removeObserver(Observer observer);

   /**
    * Removes all observers
    */
   public void removeAllObservers();
}
