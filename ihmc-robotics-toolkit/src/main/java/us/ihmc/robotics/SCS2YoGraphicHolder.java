package us.ihmc.robotics;

import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCapsule3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This interface was initially created to bind classes that can create SCS2 yoGraphics, such that:
 * <ul>
 * <li>they're easy to retrieve for refactoring purposes.
 * <li>Javadoc added here can spread around the codebase easily.
 * </ul>
 * <p>
 * SCS2 yoGraphic framework differs from SCS1 yoGraphic framework in a few aspects:
 * <ul>
 * <li>2D vs 3D:
 * <ul>
 * <li>SCS1 uses {@link YoGraphic} for 3D graphics and {@link Artifact} for 2D graphics. In this
 * documentation, we abuse the term "yoGraphic" to refer to both the 2D and 3D types when referring
 * to SCS1.
 * <li>SCS2 base class {@link YoGraphicDefinition} is extended into 2 branches:
 * {@link YoGraphic3DDefinition} for 3D graphics and {@link YoGraphic2DDefinition} for 2D graphics.
 * </ul>
 * <li>Listing and grouping:
 * <ul>
 * <li>SCS1 uses {@link YoGraphicsListRegistry} to do both listing and grouping of yoGraphics.
 * {@link YoGraphicsList} to represent a list of {@link YoGraphic} and {@link ArtifactList} for
 * {@link Artifact}. The grouping per say is done by attributing a name to a list or specifying a
 * {@code listName} when registering a yoGraphic to the {@link YoGraphicsListRegistry}.
 * <li>SCS2 yoGraphic list is {@link YoGraphicListDefinition}. It is meant for convenience, because
 * it implements {@link YoGraphicDefinition}, it can be passed around the same way a yoGraphic would
 * be. For grouping, {@link YoGraphicGroupDefinition} should be used. You can think of it as a
 * {@link YoRegistry} for yoGraphics, it has children (both regular graphics and other groups) and
 * can be added to a parent group. A tree hierarchy of yoGraphics and groups can be created and will
 * be reflected in the SCS2 GUI, see the menu <i>YoGraphic > YoGraphic properties...</i> all the
 * yoGraphics will be under the group <i>root:SessionVisualizer</i>.
 * </ul>
 * <li>Internal data:
 * <ul>
 * <li>SCS1 yoGraphics are internally backed by {@link YoVariable} which makes them harder to
 * serialize (so difficult to save to file and send over network) and also thread sensitive
 * (yoVariables should not be shared across threads).
 * <li>SCS2 yoGraphics are only descriptions telling SCS2 what type of graphic should be created and
 * what data (yoVariables or constants for instance) the graphic uses to notably change over time.
 * For instance, the radius of a capsule (see {@link YoGraphicCapsule3DDefinition}) is not a
 * yoVariable but a {@code String}. The {@code String} can be used to represent a {@code double}
 * (constant value), or a yoVariable by setting it to either {@link YoVariable#getFullNameString()}
 * or {@link YoVariable#getName()} (note that the former is preferable to account for non-uniqueness
 * of yoVariable names), SCS2 then parses the field accordingly. This makes
 * {@link YoGraphicDefinition} easier to serialize, save to file, and makes them thread-safe as no
 * state is actually held.
 * </ul>
 * <li>Graphic handling:
 * <ul>
 * <li>SCS1 yoGraphics are queried regularly by the simulation thread to update their state such
 * that the actual graphic can be updated. This is a non-trivial operation as yoGraphics often are
 * created by a controller that runs on a different thread than the simulation thread, and they
 * carry a state that is used by both the controller and simulation. To make things worse,
 * {@link Artifact} also implement the rendering.
 * <li>Because SCS2 yoGraphics do not actually carry any state, there is no such issue. This comes
 * as the cost of making it way harder if not to say impossible to hack things around from the user
 * side (outside SCS2 codebase). This often prevents implementing a workaround to a missing feature.
 * </ul>
 * <li>YoGraphics propagation to the GUI:
 * <ul>
 * <li>With SCS1 yoGraphics the typical workflow is: create a new registry
 * {@link YoGraphicsListRegistry} in the main class where SCS is created, pass it down to the
 * controller and any module running with the controller such that they can all create and register
 * their {@link YoGraphic}/{@link Artifact} to the registry, finally register the registry to SCS1
 * or a {@code YoVariableServer}. In the case where a multithread controller is used, then 1
 * registry per thread must be created and it should not be shared through threads.
 * <li>With SCS2 yoGraphics the intended workflow is: each class needing yoGraphics to be visualized
 * should declare a getter that creates and returns {@code YoGraphicDefinition}s, the calling class
 * then collects these yoGraphics and the yoGraphics of other classes it manages, and so on such
 * that the yoGraphics can bubble all the way up to where the simulation is created. The intent
 * behind that bubble up approach vs bubble down used for SCS1, is to reduce the number of
 * constructor arguments and let classes higher in the hierarchy decide which yoGraphic to
 * visualize.
 * </ul>
 * <li>Separation layer between yoGraphics and GUI rendering:
 * <ul>
 * <li>In SCS1, the yoGraphics are quite tightly connected to the rendered objects, so much that for
 * {@link Artifact}s the rendering actually happens inside the artifact class. This allows, when
 * running simulation locally, for a certain scope of changes done on the yoGraphic to be reflected
 * in the GUI even if the change is not reflected through the change in value of any yoVariable.
 * <li>In SCS2, the yoGraphics are only used at the initialization phase of the session as templates
 * to create the graphical objects. Consequently, any changes done to a yoGraphic at runtime will
 * <b>not</b> be reflected in the GUI.
 * </ul>
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface SCS2YoGraphicHolder
{
   /**
    * The intended implementation for this method is:
    * <ul>
    * <li>to create and return the yoGraphics to be visualized,
    * <li>to collect and return the yoGraphics from the classes managed by the class.
    * </ul>
    * <p>
    * Useful notes:
    * <ul>
    * <li>Use {@link YoGraphicGroupDefinition} to gather multiple yoGraphics and other groups into a
    * single {@link YoGraphicDefinition}. The group will appear as a named registry in the SCS GUI.
    * Including the name of the class in the group being created helps locating yoGraphics in the code.
    * <li>Use {@link YoGraphicListDefinition} if you do not want to create a group but still need to
    * gather multiple yoGraphics into a single {@link YoGraphicDefinition}.
    * <li>Some helper classes:
    * <ul>
    * <li>{@link YoGraphicDefinitionFactory}: gather convenience methods to create
    * {@link YoGraphicDefinition}s and other types needed to create yoGraphic.
    * <li>{@link YoGraphicConversionTools}: for conversion tools between SCS1 and SCS2.
    * <li>{@link SCS2DefinitionMissingTools}: for implementing that method that should have been
    * implemented somewhere in {@code scs2-definition}.
    * </ul>
    * </ul>
    * </p>
    * 
    * @return the yoGraphics to be visualized in the SCS GUI.
    */
   YoGraphicDefinition getSCS2YoGraphics();
}
