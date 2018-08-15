package us.ihmc.robotics.stateMachine.extra;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.swing.JPanel;

import org.jgraph.JGraph;
import org.jgraph.graph.DefaultCellViewFactory;
import org.jgraph.graph.DefaultEdge;
import org.jgraph.graph.DefaultGraphCell;
import org.jgraph.graph.DefaultGraphModel;
import org.jgraph.graph.GraphConstants;
import org.jgraph.graph.GraphLayoutCache;

import com.jgraph.layout.JGraphFacade;
import com.jgraph.layout.tree.JGraphTreeLayout;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;

/**
 * This is an old piece code that has been upgraded to work with the new {@link StateMachine}.
 * <p>
 * It will some major clean up.
 * </p>
 * <p>
 * {@link StateMachinesJPanel} can be use to create a new {@code JPanel} displaying a graphical
 * representation of a state machine.
 * </p>
 *
 * @author Sylvain
 *
 * @param <K> Type of {@link Enum} that lists the potential states and that is used by the state
 *           machine this listener is to be added to.
 */
public class StateMachinesJPanel<K extends Enum<K>> extends JPanel implements StateChangedListener<K>
{
   private static final long serialVersionUID = 2453853798153829891L;
   private final StateMachine<K, ? extends State> stateMachine;
   private boolean oldStateDiagram;
   private final Map<K, DefaultGraphCell> stateCells;
   private final Set<K> stateKeys;
   private final Map<K, State> states;
   private final Map<K, StateTransition<K>> stateTransitions;

   private final Class<K> keyType;
   private final Map<K, Set<K>> arrows;

   private JGraph graph;
   private final int numberOfStates;
   private K currentStateKey = null;

   /**
    * Creates a new graphic representation of the given state machine.
    *
    * @param stateMachine the state machine to be graphically displayed.
    */
   public StateMachinesJPanel(StateMachine<K, ? extends State> stateMachine)
   {
      this(stateMachine, false);
   }

   /**
    * Creates a new graphic representation of the given state machine.
    *
    * @param stateMachine the state machine to be graphically displayed.
    * @param oldStateDiagram when {@code true} each state is displayed as a circle and the states are
    *           organized on circular pattern, {@code false} each state is displayed as a box and the
    *           states are organized with a linear pattern.
    */
   public StateMachinesJPanel(StateMachine<K, ? extends State> stateMachine, boolean oldStateDiagram)
   {
      this.stateMachine = stateMachine;
      currentStateKey = stateMachine.getCurrentStateKey();
      keyType = stateMachine.getStateKeyType();
      states = extractActiveStates(stateMachine);
      stateTransitions = extractActiveStateTransitions(stateMachine);
      stateKeys = states.keySet();
      this.numberOfStates = states.size();
      this.oldStateDiagram = oldStateDiagram;
      stateCells = new EnumMap<>(keyType);
      arrows = new EnumMap<>(keyType);

      if (!oldStateDiagram)
      {
         graph = new JGraph();
         initializeJPanel();
      }

      Thread repaintWhenStateChange = new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            while (true)
            {
               // System.out.println("painting");
               try
               {
                  Thread.sleep(100);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }

               if (currentStateKey != stateMachine.getCurrentStateKey())
               {
                  currentStateKey = stateMachine.getCurrentStateKey();
                  updateStateMachine();
               }
            }
         }
      }, "IHMC-StateMachinesPanelUpdater");
      repaintWhenStateChange.start();

   }

   private static <K extends Enum<K>> Map<K, StateTransition<K>> extractActiveStateTransitions(StateMachine<K, ? extends State> stateMachine)
   {
      Class<K> keyType = stateMachine.getStateKeyType();
      K[] allKeys = keyType.getEnumConstants();
      Map<K, StateTransition<K>> states = new EnumMap<>(keyType);

      for (K key : allKeys)
      {
         StateTransition<K> state = stateMachine.getStateTransition(key);
         if (state != null)
            states.put(key, state);
      }
      return states;
   }

   private static <K extends Enum<K>> Map<K, State> extractActiveStates(StateMachine<K, ? extends State> stateMachine)
   {
      Class<K> keyType = stateMachine.getStateKeyType();
      K[] allKeys = keyType.getEnumConstants();
      Map<K, State> states = new EnumMap<>(keyType);

      for (K key : allKeys)
      {
         State state = stateMachine.getState(key);
         if (state != null)
            states.put(key, state);
      }
      return states;
   }

   private void updateStateMachine()
   {
      if (oldStateDiagram)
      {
         this.repaint();
      }
      else
      {
         K currentStateKey = stateMachine.getCurrentStateKey();
         DefaultGraphCell currentStateCell = stateCells.get(currentStateKey);
         GraphConstants.setBackground(graph.getAttributes(currentStateCell), Color.RED);
         currentStateCell.setAttributes(graph.getAttributes(currentStateCell));

         for (Entry<K, DefaultGraphCell> entry : stateCells.entrySet())
         {
            if (entry.getKey() != currentStateKey)
            {
               DefaultGraphCell stateCell = entry.getValue();
               GraphConstants.setBackground(graph.getAttributes(stateCell), Color.CYAN);
               stateCell.setAttributes(graph.getAttributes(stateCell));
            }
         }

         graph.repaint();
         graph.refresh();

      }
   }

   @Override
   public void stateChanged(K oldState, K newState)
   {
      if (oldStateDiagram)
      {
         this.repaint();
      }
      else
      {
         if (oldState != null)
         { // At initialization, the state key is null.
            DefaultGraphCell oldStateCell = stateCells.get(oldState);
            GraphConstants.setBackground(graph.getAttributes(oldStateCell), Color.CYAN);
            oldStateCell.setAttributes(graph.getAttributes(oldStateCell));
         }

         DefaultGraphCell newStateCell = stateCells.get(newState);
         GraphConstants.setBackground(graph.getAttributes(newStateCell), Color.RED);
         newStateCell.setAttributes(graph.getAttributes(newStateCell));

         graph.repaint();
         this.repaint();
         graph.refresh();
      }
   }

   private void createArrow(K source, K target)
   {
      if (!arrowAlreadyExist(source, target))
      {

         DefaultGraphCell[] arrowCell = new DefaultGraphCell[1];
         DefaultEdge edge = new DefaultEdge();
         edge.setSource(stateCells.get(source).getChildAt(0));
         edge.setTarget(stateCells.get(target).getChildAt(0));
         arrowCell[0] = edge;
         int arrow = GraphConstants.ARROW_TECHNICAL;
         GraphConstants.setLineEnd(edge.getAttributes(), arrow);
         GraphConstants.setEndFill(edge.getAttributes(), true);
         graph.getGraphLayoutCache().insert(arrowCell);

         Set<K> targetSet = arrows.get(source);
         if (targetSet == null)
         {
            targetSet = EnumSet.noneOf(keyType);
            arrows.put(source, targetSet);
         }
         targetSet.add(target);
      }
   }

   private boolean arrowAlreadyExist(K source, K target)
   {
      return arrows.containsKey(source) && arrows.get(source).contains(target);
   }

   private DefaultGraphCell createCell(K stateKey, Point2D placement)
   {
      //      System.out.println(name + " " + placement + " " + index);
      String cellName = stateKey.toString();
      DefaultGraphCell stateCell = new DefaultGraphCell(new String(cellName));
      Font font = new Font("Arial", Font.PLAIN, 12);
      GraphConstants.setFont(stateCell.getAttributes(), font);
      GraphConstants.setAutoSize(stateCell.getAttributes(), true);
      GraphConstants.setOpaque(stateCell.getAttributes(), true);

      Color color = colorStateCell(stateKey);
      GraphConstants.setBackground(stateCell.getAttributes(), color);
      GraphConstants.setBounds(stateCell.getAttributes(), new Rectangle2D.Double(placement.getX(), placement.getY(), 0, 0));
      stateCell.addPort();

      return stateCell;
   }

   private void createStateTransitionDiagram(K sourceStateKey)
   {
      StateTransition<K> parentStateTransition = stateTransitions.get(sourceStateKey);

      for (K endStateKey : parentStateTransition)
      {
         createArrow(sourceStateKey, endStateKey);
      }
   }

   private Color colorStateCell(K stateKey)
   {
      if (stateKey == stateMachine.getCurrentStateKey())
      {
         return Color.RED;
      }
      else
      {
         return Color.CYAN;
      }
   }

   private void initializeJPanel()
   {
      initiate = true;
      graph.setGraphLayoutCache(new GraphLayoutCache(new DefaultGraphModel(), new DefaultCellViewFactory(), true));
      graph.setEditable(false);

      for (K key : stateKeys)
         stateCells.put(key, createCell(key, new Point2D(0, 0)));

      for (K key : stateKeys)
         createStateTransitionDiagram(key);

      graph.getGraphLayoutCache().insert(stateCells.values().toArray());

      JGraphFacade facade = new JGraphFacade(graph.getGraphLayoutCache());
      facade.setEdgePromotion(false);
      JGraphTreeLayout layout = new JGraphTreeLayout();
      layout.run(facade);
      Map<?, ?> map = facade.createNestedMap(true, true);
      graph.getGraphLayoutCache().edit(map);
   }

   private Dimension dimension = new Dimension();
   final static BasicStroke wideStroke = new BasicStroke(3.0f);

   private void oldStateMachineDiagram(Graphics graphics, double width, double height)
   {
      Graphics2D graphic2D = (Graphics2D) graphics;
      graphic2D.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      graphic2D.setStroke(wideStroke);

      // Determine the Ellipse Radii and Circumference:
      double Ry = 0.25 * height;
      double Rx = 0.25 * width;
      double circumference = approximateElipseCircumference(Rx, Ry);

      // Determine the Circle Radius:
      int circleRadiusBasedOnSpacing = (int) (circumference / (2.5 * numberOfStates));
      int circleRadiusBasedOnPanelSize = (int) (0.9 * Math.min(Rx, Ry));
      int circleRadius = Math.min(circleRadiusBasedOnSpacing, circleRadiusBasedOnPanelSize);

      // Draw a circle for each state:
      K currentStateKey = stateMachine.getCurrentStateKey();

      int stateIndex = 0;
      Map<K, Point2D> stateCenters = new EnumMap<>(keyType);

      for (K stateKey : stateKeys)
      {
         Point2D stateCenter = new Point2D();
         if (stateKey == currentStateKey)
            graphics.setColor(Color.RED);
         else
            graphic2D.setColor(Color.BLACK);

         double angle = 2.0 * Math.PI * stateIndex / numberOfStates;
         stateCenter.setX(Rx * Math.cos(angle) + width / 2.0);
         stateCenter.setY(Ry * Math.sin(angle) + height / 2.0);

         graphic2D.draw(new Ellipse2D.Double(stateCenter.getX() - circleRadius, stateCenter.getY() - circleRadius, circleRadius * 2, circleRadius * 2));
         String stateString = stateKey.toString();

         graphic2D.drawString(stateString, (int) stateCenter.getX() - 8 * stateString.length() / 2, (int) stateCenter.getY());
         stateIndex++;

         stateCenters.put(stateKey, stateCenter);
      }

      graphic2D.setColor(Color.BLACK);

      // Draw a line for each transition condition:
      for (K startKey : stateKeys)
      {
         StateTransition<K> stateTransition = stateTransitions.get(startKey);

         for (K endKey : stateTransition)
         {
            stateMachine.getState(endKey); // Just to make sure there is a state.

            Vector2D direction = new Vector2D();
            Point2D startCenter = stateCenters.get(startKey);
            Point2D endCenter = stateCenters.get(endKey);
            direction.sub(endCenter, startCenter);
            direction.normalize();
            direction.scale(circleRadius);

            double x1 = startCenter.getX() + direction.getX();
            double y1 = startCenter.getY() + direction.getY();
            double x2 = endCenter.getX() - direction.getX();
            double y2 = endCenter.getY() - direction.getY();
            graphic2D.draw(new Line2D.Double(x1, y1, x2, y2));
         }
      }
   }

   private boolean initiate;
   private double tempHeight = 0;
   private double tempWidth = 0;

   private void reScalingStateMachine(double width, double height)
   {
      if (initiate)
      {
         tempWidth = width;
         tempHeight = height;
         initiate = false;
      }

      if (tempWidth > width)
      {
         graph.setScale(graph.getScale() + (width - tempWidth) / 3000);
         tempWidth = width;
      }

      if (tempWidth < width)
      {
         graph.setScale(graph.getScale() + (width - tempWidth) / 3000);
         tempWidth = width;
      }

      if (tempHeight > height)
      {
         graph.setScale(graph.getScale() - (tempHeight - height) / 3000);
         tempHeight = height;
      }

      if (tempHeight < height)
      {
         graph.setScale(graph.getScale() + (height - tempHeight) / 3000);
         tempHeight = height;
      }
   }

   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      this.getSize(dimension);
      double width = dimension.getWidth();
      double height = dimension.getHeight();

      if (oldStateDiagram)
      {
         oldStateMachineDiagram(g, width, height);
      }
      else
      {
         graph.setBounds(0, 0, (int) width, (int) height);
         graph.setAlignmentX(CENTER_ALIGNMENT);
         reScalingStateMachine(width, height);
         this.add(graph);
      }
   }

   private static double approximateElipseCircumference(double a, double b)
   {
      double x = (a - b) / (a + b);

      return Math.PI * (a + b) * (1.0 + 3.0 * x * x / (10.0 + Math.sqrt(4.0 - 3.0 * x * x)));
   }
}
