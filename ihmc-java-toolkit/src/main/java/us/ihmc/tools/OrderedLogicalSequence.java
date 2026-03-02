package us.ihmc.tools;

import us.ihmc.yoVariables.providers.BooleanProvider;

import java.util.ArrayList;

/**
 * Simple tool to assemble an ordered sequence of runnable logical elements that must be
 * executed in a specific order. Each logical element in the list has a corresponding
 * prerequisite logical condition that must be true in order to begin the logic in the
 * given element, and a completion logical condition that must be true in order to move
 * on to the next element in the list
 *
 * @author Stefan Fasano
 */
public class OrderedLogicalSequence
{
   private final ArrayList<Runnable> logicalElements = new ArrayList<>();
   private final ArrayList<BooleanProvider> preRequisiteConditions = new ArrayList<>();
   private final ArrayList<BooleanProvider> completionConditions = new ArrayList<>();

   private int currentLogicalElement = 0;
   private boolean started = false;
   private boolean finished = false;

   public OrderedLogicalSequence()
   {
   }

   /**
    * Adds a logical element to this ordered logical sequence
    * @param runnable---------------The main logic to run
    * @param preRequisiteCondition--Prerequisite logic that must be true before running main logic
    * @param completionCondition----Completion logic that must be true before moving to the next element
    */
   public void addLogicalElement(Runnable runnable, BooleanProvider preRequisiteCondition, BooleanProvider completionCondition)
   {
      logicalElements.add(runnable);
      preRequisiteConditions.add(preRequisiteCondition);
      completionConditions.add(completionCondition);
   }

   /**
    * Calling this will allow execution of this ordered logical sequence to begin
    */
   public void start()
   {
      started = true;
   }

   /**
    * Resets this ordered logical sequence so it can be used again
    */
   public void reset()
   {
      started = false;
      finished = false;
      currentLogicalElement = 0;
   }

   /**
    * Main update loop of this ordered logical sequence
    */
   public void update()
   {
      // If the sequence has not been started, or if it is finished, skip this update
      if (!started || finished)
         return;

      // If the prerequisite condition has been met for this element, run the runnable logic of this element
      if (preRequisiteConditions.get(currentLogicalElement) != null && preRequisiteConditions.get(currentLogicalElement).getValue())
         logicalElements.get(currentLogicalElement).run();

      // If the completion condition has been met for this element, increment to the next element
      if (completionConditions.get(currentLogicalElement) != null && completionConditions.get(currentLogicalElement).getValue())
         currentLogicalElement++;

      // If we have incremented to the end of the list of elements, we are finished
      if (currentLogicalElement >= logicalElements.size() - 1)
         finished = true;
   }

   /**
    * @return {@link OrderedLogicalSequence#started}: if the logical sequence has started
    */
   public boolean hasStarted()
   {
      return started;
   }

   /**
    * @return {@link OrderedLogicalSequence#finished}: if the logical sequence has finished
    */
   public boolean hasFinished()
   {
      return finished;
   }
}
